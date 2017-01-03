/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author: Yakir Yang <ykk@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/input.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_psr.h"

#define PSR_FLUSH_TIMEOUT_MS		100

enum psr_state {
	PSR_FLUSH,
	PSR_ENABLE,
	PSR_DISABLE,
};

struct psr_drv {
	struct list_head	list;
	struct drm_encoder	*encoder;

	struct mutex		lock;
	bool			active;
	enum psr_state		state;

	struct delayed_work	flush_work;
	struct work_struct	disable_work;

	struct input_handler    input_handler;

	int (*set)(struct drm_encoder *encoder, bool enable);
};

static struct psr_drv *find_psr_by_crtc(struct drm_crtc *crtc)
{
	struct rockchip_drm_private *drm_drv = crtc->dev->dev_private;
	struct psr_drv *psr;

	mutex_lock(&drm_drv->psr_list_lock);
	list_for_each_entry(psr, &drm_drv->psr_list, list) {
		if (psr->encoder->crtc == crtc)
			goto out;
	}
	psr = ERR_PTR(-ENODEV);

out:
	mutex_unlock(&drm_drv->psr_list_lock);
	return psr;
}

static struct psr_drv *find_psr_by_encoder(struct drm_encoder *encoder)
{
	struct rockchip_drm_private *drm_drv = encoder->dev->dev_private;
	struct psr_drv *psr;

	mutex_lock(&drm_drv->psr_list_lock);
	list_for_each_entry(psr, &drm_drv->psr_list, list) {
		if (psr->encoder == encoder)
			goto out;
	}
	psr = ERR_PTR(-ENODEV);

out:
	mutex_unlock(&drm_drv->psr_list_lock);
	return psr;
}

static void psr_set_state_locked(struct psr_drv *psr, enum psr_state state)
{
	/*
	 * Allowed finite state machine:
	 *
	 *   PSR_ENABLE  < = = = = = >  PSR_FLUSH
	 *       | ^                        |
	 *       | |                        |
	 *       v |                        |
	 *   PSR_DISABLE < - - - - - - - - -
	 */
	if (state == psr->state || !psr->active)
		return;

	/* Already disabled in flush, change the state, but not the hardware */
	if (state == PSR_DISABLE && psr->state == PSR_FLUSH) {
		psr->state = state;
		return;
	}

	/* Actually commit the state change to hardware */
	switch (state) {
	case PSR_ENABLE:
		if (psr->set(psr->encoder, true))
			return;
		break;

	case PSR_DISABLE:
	case PSR_FLUSH:
		if (psr->set(psr->encoder, false))
			return;
		break;

	default:
		pr_err("%s: Unknown state %d\n", __func__, state);
		return;
	}

	psr->state = state;
}

static void psr_set_state(struct psr_drv *psr, enum psr_state state)
{
	mutex_lock(&psr->lock);
	psr_set_state_locked(psr, state);
	mutex_unlock(&psr->lock);
}

static void psr_flush_handler(struct work_struct *work)
{
	struct psr_drv *psr = container_of(to_delayed_work(work),
					   struct psr_drv, flush_work);

	/* If the state has changed since we initiated the flush, do nothing */
	mutex_lock(&psr->lock);
	if (psr->state == PSR_FLUSH)
		psr_set_state_locked(psr, PSR_ENABLE);
	mutex_unlock(&psr->lock);
}

static void psr_disable_handler(struct work_struct *work)
{
	struct psr_drv *psr = container_of(work, struct psr_drv, disable_work);

	/* If the state has changed since we initiated the flush, do nothing */
	mutex_lock(&psr->lock);
	if (psr->state == PSR_ENABLE)
		psr_set_state_locked(psr, PSR_FLUSH);
	mutex_unlock(&psr->lock);
	mod_delayed_work(system_wq, &psr->flush_work, PSR_FLUSH_TIMEOUT_MS);
}

/**
 * rockchip_drm_psr_activate - activate PSR on the given pipe
 * @encoder: encoder to obtain the PSR encoder
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int rockchip_drm_psr_activate(struct drm_encoder *encoder)
{
	struct psr_drv *psr = find_psr_by_encoder(encoder);

	if (IS_ERR(psr))
		return PTR_ERR(psr);

	mutex_lock(&psr->lock);
	psr->active = true;
	mutex_unlock(&psr->lock);

	return 0;
}
EXPORT_SYMBOL(rockchip_drm_psr_activate);

/**
 * rockchip_drm_psr_deactivate - deactivate PSR on the given pipe
 * @encoder: encoder to obtain the PSR encoder
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int rockchip_drm_psr_deactivate(struct drm_encoder *encoder)
{
	struct psr_drv *psr = find_psr_by_encoder(encoder);

	if (IS_ERR(psr))
		return PTR_ERR(psr);

	mutex_lock(&psr->lock);
	psr->active = false;
	mutex_unlock(&psr->lock);
	cancel_delayed_work_sync(&psr->flush_work);
	cancel_work_sync(&psr->disable_work);

	return 0;
}
EXPORT_SYMBOL(rockchip_drm_psr_deactivate);

static void rockchip_drm_do_flush(struct psr_drv *psr)
{
	psr_set_state(psr, PSR_FLUSH);
	mod_delayed_work(system_wq, &psr->flush_work, PSR_FLUSH_TIMEOUT_MS);
}

/**
 * rockchip_drm_psr_flush - flush a single pipe
 * @crtc: CRTC of the pipe to flush
 *
 * Returns:
 * 0 on success, -errno on fail
 */
int rockchip_drm_psr_flush(struct drm_crtc *crtc)
{
	struct psr_drv *psr = find_psr_by_crtc(crtc);
	if (IS_ERR(psr))
		return PTR_ERR(psr);

	rockchip_drm_do_flush(psr);
	return 0;
}
EXPORT_SYMBOL(rockchip_drm_psr_flush);

/**
 * rockchip_drm_psr_flush_all - force to flush all registered PSR encoders
 * @dev: drm device
 *
 * Disable the PSR function for all registered encoders, and then enable the
 * PSR function back after PSR_FLUSH_TIMEOUT. If encoder PSR state have been
 * changed during flush time, then keep the state no change after flush
 * timeout.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
void rockchip_drm_psr_flush_all(struct drm_device *dev)
{
	struct rockchip_drm_private *drm_drv = dev->dev_private;
	struct psr_drv *psr;

	mutex_lock(&drm_drv->psr_list_lock);
	list_for_each_entry(psr, &drm_drv->psr_list, list)
		rockchip_drm_do_flush(psr);
	mutex_unlock(&drm_drv->psr_list_lock);
}
EXPORT_SYMBOL(rockchip_drm_psr_flush_all);

static void psr_input_event(struct input_handle *handle,
			    unsigned int type, unsigned int code,
			    int value)
{
	struct psr_drv *psr = handle->handler->private;

	schedule_work(&psr->disable_work);
}

static int psr_input_connect(struct input_handler *handler,
			     struct input_dev *dev,
			     const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "rockchip-psr";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;

err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void psr_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

/* Same device ids as cpu-boost */
static const struct input_device_id psr_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			    BIT_MASK(ABS_MT_POSITION_X) |
			    BIT_MASK(ABS_MT_POSITION_Y) },
	}, /* multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_X)] = BIT_MASK(ABS_X) }

	}, /* stylus or joystick device */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = { [BIT_WORD(BTN_LEFT)] = BIT_MASK(BTN_LEFT) },
	}, /* pointer (e.g. trackpad, mouse) */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = { [BIT_WORD(KEY_ESC)] = BIT_MASK(KEY_ESC) },
	}, /* keyboard */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
				INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = {[BIT_WORD(BTN_JOYSTICK)] = BIT_MASK(BTN_JOYSTICK) },
	}, /* joysticks not caught by ABS_X above */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
				INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = { [BIT_WORD(BTN_GAMEPAD)] = BIT_MASK(BTN_GAMEPAD) },
	}, /* gamepad */
	{ },
};

/**
 * rockchip_drm_psr_register - register encoder to psr driver
 * @encoder: encoder that obtain the PSR function
 * @psr_set: call back to set PSR state
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int rockchip_drm_psr_register(struct drm_encoder *encoder,
			int (*psr_set)(struct drm_encoder *, bool enable))
{
	struct rockchip_drm_private *drm_drv = encoder->dev->dev_private;
	struct psr_drv *psr;
	int error;

	if (!encoder || !psr_set)
		return -EINVAL;

	psr = kzalloc(sizeof(struct psr_drv), GFP_KERNEL);
	if (!psr)
		return -ENOMEM;

	INIT_DELAYED_WORK(&psr->flush_work, psr_flush_handler);
	INIT_WORK(&psr->disable_work, psr_disable_handler);
	mutex_init(&psr->lock);

	psr->active = true;
	psr->state = PSR_DISABLE;
	psr->encoder = encoder;
	psr->set = psr_set;

	psr->input_handler.event = psr_input_event;
	psr->input_handler.connect = psr_input_connect;
	psr->input_handler.disconnect = psr_input_disconnect;
	psr->input_handler.name =
		kasprintf(GFP_KERNEL, "rockchip-psr-%s", encoder->name);
	if (!psr->input_handler.name) {
		error = -ENOMEM;
		goto err2;
	}
	psr->input_handler.id_table = psr_ids;
	psr->input_handler.private = psr;

	error = input_register_handler(&psr->input_handler);
	if (error)
		goto err1;

	mutex_lock(&drm_drv->psr_list_lock);
	list_add_tail(&psr->list, &drm_drv->psr_list);
	mutex_unlock(&drm_drv->psr_list_lock);

	return 0;

 err1:
	kfree(psr->input_handler.name);
 err2:
	kfree(psr);
	return error;
}
EXPORT_SYMBOL(rockchip_drm_psr_register);

/**
 * rockchip_drm_psr_unregister - unregister encoder to psr driver
 * @encoder: encoder that obtain the PSR function
 * @psr_set: call back to set PSR state
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
void rockchip_drm_psr_unregister(struct drm_encoder *encoder)
{
	struct rockchip_drm_private *drm_drv = encoder->dev->dev_private;
	struct psr_drv *psr, *n;

	mutex_lock(&drm_drv->psr_list_lock);
	list_for_each_entry_safe(psr, n, &drm_drv->psr_list, list) {
		if (psr->encoder == encoder) {
			input_unregister_handler(&psr->input_handler);
			cancel_delayed_work_sync(&psr->flush_work);
			cancel_work_sync(&psr->disable_work);
			list_del(&psr->list);
			kfree(psr->input_handler.name);
			kfree(psr);
		}
	}
	mutex_unlock(&drm_drv->psr_list_lock);
}
EXPORT_SYMBOL(rockchip_drm_psr_unregister);
