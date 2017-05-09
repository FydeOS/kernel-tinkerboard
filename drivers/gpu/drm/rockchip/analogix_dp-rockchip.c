/*
 * Rockchip SoC DP (Display Port) interface driver.
 *
 * Copyright (C) Fuzhou Rockchip Electronics Co., Ltd.
 * Author: Andy Yan <andy.yan@rock-chips.com>
 *         Yakir Yang <ykk@rock-chips.com>
 *         Jeff Chen <jeff.chen@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/component.h>
#include <linux/debugfs.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/clk.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>

#include <video/of_videomode.h>
#include <video/videomode.h>

#include <drm/bridge/analogix_dp.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_psr.h"
#include "rockchip_drm_vop.h"

#define PSR_WAIT_LINE_FLAG_TIMEOUT_MS	100

#define to_dp(nm)	container_of(nm, struct rockchip_dp_device, nm)

/**
 * struct rockchip_dp_chip_data - splite the grf setting of kind of chips
 * @lcdsel_grf_reg: grf register offset of lcdc select
 * @lcdsel_big: reg value of selecting vop big for eDP
 * @lcdsel_lit: reg value of selecting vop little for eDP
 */
struct rockchip_dp_chip_data {
	u32	lcdsel_grf_reg;
	u32	lcdsel_big;
	u32	lcdsel_lit;
	u32	chip_type;
};

struct rockchip_dp_device {
	struct drm_device        *drm_dev;
	struct device            *dev;
	struct drm_encoder       encoder;
	struct drm_display_mode  mode;

	struct clk               *pclk;
	struct clk               *grfclk;
	struct regmap            *grf;
	struct reset_control     *rst;

	const struct rockchip_dp_chip_data *data;

	struct analogix_dp_device *adp;
	struct analogix_dp_plat_data plat_data;

	struct dentry            *debugfs_psr;
	u64                      psr_active_ms;
	ktime_t                  psr_start_time;
};

static int analogix_dp_psr_set(struct drm_encoder *encoder, bool enabled)
{
	struct rockchip_dp_device *dp = to_dp(encoder);
	struct drm_crtc *crtc = dp->encoder.crtc;
	s64 delta_ms;
	int ret;

	dev_dbg(dp->dev, "%s PSR...\n", enabled ? "enable" : "disable");

	if (!crtc)
		return -EINVAL;

	if (enabled) {
		ret = rockchip_drm_wait_vact_end(crtc,
						 PSR_WAIT_LINE_FLAG_TIMEOUT_MS);
		if (ret) {
			dev_err(dp->dev, "line flag interrupt did not arrive\n");
			return -ETIMEDOUT;
		}

		ret = analogix_dp_enable_psr(dp->adp);
		if (ret) {
			dev_err(dp->dev, "failed to enable psr %d\n", ret);
			return ret;
		}
		rockchip_drm_set_win_enabled(crtc, false);
		dp->psr_start_time = ktime_get();
	} else {
		rockchip_drm_set_win_enabled(crtc, true);
		ret = analogix_dp_disable_psr(dp->adp);
		if (ret) {
			dev_err(dp->dev, "failed to disable psr %d\n", ret);
			return ret;
		}
		delta_ms = ktime_ms_delta(ktime_get(), dp->psr_start_time);
		if (!WARN_ON(delta_ms < 0))
			dp->psr_active_ms += delta_ms;
	}
	return 0;
}

static int rockchip_dp_pre_init(struct rockchip_dp_device *dp)
{
	reset_control_assert(dp->rst);
	usleep_range(10, 20);
	reset_control_deassert(dp->rst);

	return 0;
}

static int rockchip_dp_poweron_start(struct analogix_dp_plat_data *plat_data)
{
	struct rockchip_dp_device *dp = to_dp(plat_data);
	int ret;

	ret = clk_prepare_enable(dp->pclk);
	if (ret < 0) {
		dev_err(dp->dev, "failed to enable pclk %d\n", ret);
		return ret;
	}

	ret = rockchip_dp_pre_init(dp);
	if (ret < 0)
		dev_err(dp->dev, "failed to dp pre init %d\n", ret);

	return ret;
}

static int rockchip_dp_poweron_end(struct analogix_dp_plat_data *plat_data)
{
	struct rockchip_dp_device *dp = to_dp(plat_data);

	if (!analogix_dp_is_psr_available(dp->adp))
		return 0;

	return rockchip_drm_psr_inhibit_put(&dp->encoder);
}

static int rockchip_dp_powerdown(struct analogix_dp_plat_data *plat_data)
{
	struct rockchip_dp_device *dp = to_dp(plat_data);
	int ret;

	if (analogix_dp_is_psr_available(dp->adp)) {
		ret = rockchip_drm_psr_inhibit_get(&dp->encoder);
		if (ret != 0)
			return ret;
	}

	clk_disable_unprepare(dp->pclk);

	return 0;
}

static int rockchip_dp_get_modes(struct analogix_dp_plat_data *plat_data,
				 struct drm_connector *connector)
{
	struct drm_display_info *di = &connector->display_info;

	if (di->color_formats & DRM_COLOR_FORMAT_YCRCB444 ||
	    di->color_formats & DRM_COLOR_FORMAT_YCRCB422) {
		di->color_formats &= ~(DRM_COLOR_FORMAT_YCRCB422 |
				       DRM_COLOR_FORMAT_YCRCB444);
		di->color_formats |= DRM_COLOR_FORMAT_RGB444;
		di->bpc = 8;
	}

	return 0;
}

static bool
rockchip_dp_drm_encoder_mode_fixup(struct drm_encoder *encoder,
				   const struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	/* do nothing */
	return true;
}

static void rockchip_dp_drm_encoder_mode_set(struct drm_encoder *encoder,
					     struct drm_display_mode *mode,
					     struct drm_display_mode *adjusted)
{
	/* do nothing */
}

static void rockchip_dp_drm_encoder_enable(struct drm_encoder *encoder)
{
	struct rockchip_dp_device *dp = to_dp(encoder);
	int ret;
	u32 val;

	switch (vop_get_crtc_vop_id(encoder->crtc)) {
	case RK3399_VOP_LIT:
		val = dp->data->lcdsel_lit;
		break;
	case RK3399_VOP_BIG:
		val = dp->data->lcdsel_big;
		break;
	default:
		return;
	}

	dev_dbg(dp->dev, "vop %s output to dp\n", (ret) ? "LIT" : "BIG");

	ret = clk_prepare_enable(dp->grfclk);
	if (ret < 0) {
		dev_err(dp->dev, "failed to enable grfclk %d\n", ret);
		return;
	}

	ret = regmap_write(dp->grf, dp->data->lcdsel_grf_reg, val);
	if (ret != 0)
		dev_err(dp->dev, "Could not write to GRF: %d\n", ret);

	clk_disable_unprepare(dp->grfclk);
}

static void rockchip_dp_drm_encoder_nop(struct drm_encoder *encoder)
{
	/* do nothing */
}

static int
rockchip_dp_drm_encoder_atomic_check(struct drm_encoder *encoder,
				      struct drm_crtc_state *crtc_state,
				      struct drm_connector_state *conn_state)
{
	struct rockchip_crtc_state *s = to_rockchip_crtc_state(crtc_state);
	struct drm_display_info *di = &conn_state->connector->display_info;

	switch (vop_get_crtc_vop_id(crtc_state->crtc)) {
	case RK3399_VOP_LIT:
		/*
		 * For RK3399, VOP Lit must code the out mode to RGB888,
		 * VOP Big must code the out mode to RGB10.
		 */
		s->output_mode = ROCKCHIP_OUT_MODE_P888;
		break;
	default:
		/*
		 * The hardware IC designed that VOP must output the RGB10 video
		 * format to eDP contoller, and if eDP panel only support RGB8,
		 * then eDP contoller should cut down the video data, not via VOP
		 * contoller, that's why we need to hardcode the VOP output mode
		 * to RGA10 here.
		 */
		s->output_mode = ROCKCHIP_OUT_MODE_AAAA;
		break;
	}

	s->output_bpc = di->bpc;
	s->output_type = DRM_MODE_CONNECTOR_eDP;

	return 0;
}

static struct drm_encoder_helper_funcs rockchip_dp_encoder_helper_funcs = {
	.mode_fixup = rockchip_dp_drm_encoder_mode_fixup,
	.mode_set = rockchip_dp_drm_encoder_mode_set,
	.enable = rockchip_dp_drm_encoder_enable,
	.disable = rockchip_dp_drm_encoder_nop,
	.atomic_check = rockchip_dp_drm_encoder_atomic_check,
};

static void rockchip_dp_drm_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static struct drm_encoder_funcs rockchip_dp_encoder_funcs = {
	.destroy = rockchip_dp_drm_encoder_destroy,
};

static int rockchip_dp_init(struct rockchip_dp_device *dp)
{
	struct device *dev = dp->dev;
	struct device_node *np = dev->of_node;
	int ret;

	dp->grf = syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR(dp->grf)) {
		dev_err(dev, "failed to get rockchip,grf property\n");
		return PTR_ERR(dp->grf);
	}

	dp->grfclk = devm_clk_get(dev, "grf");
	if (PTR_ERR(dp->grfclk) == -ENOENT) {
		dp->grfclk = NULL;
	} else if (PTR_ERR(dp->grfclk) == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	} else if (IS_ERR(dp->grfclk)) {
		dev_err(dev, "failed to get grf clock\n");
		return PTR_ERR(dp->grfclk);
	}

	dp->pclk = devm_clk_get(dev, "pclk");
	if (IS_ERR(dp->pclk)) {
		dev_err(dev, "failed to get pclk property\n");
		return PTR_ERR(dp->pclk);
	}

	dp->rst = devm_reset_control_get(dev, "dp");
	if (IS_ERR(dp->rst)) {
		dev_err(dev, "failed to get dp reset control\n");
		return PTR_ERR(dp->rst);
	}

	ret = clk_prepare_enable(dp->pclk);
	if (ret < 0) {
		dev_err(dp->dev, "failed to enable pclk %d\n", ret);
		return ret;
	}

	ret = rockchip_dp_pre_init(dp);
	if (ret < 0) {
		dev_err(dp->dev, "failed to pre init %d\n", ret);
		return ret;
	}

	return 0;
}

static int rockchip_dp_drm_create_encoder(struct rockchip_dp_device *dp)
{
	struct drm_encoder *encoder = &dp->encoder;
	struct drm_device *drm_dev = dp->drm_dev;
	struct device *dev = dp->dev;
	int ret;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm_dev,
							     dev->of_node);
	DRM_DEBUG_KMS("possible_crtcs = 0x%x\n", encoder->possible_crtcs);

	ret = drm_encoder_init(drm_dev, encoder, &rockchip_dp_encoder_funcs,
			       DRM_MODE_ENCODER_TMDS, NULL);
	if (ret) {
		DRM_ERROR("failed to initialize encoder with drm\n");
		return ret;
	}

	drm_encoder_helper_add(encoder, &rockchip_dp_encoder_helper_funcs);

	return 0;
}

static int rockchip_dp_bind(struct device *dev, struct device *master,
			    void *data)
{
	struct rockchip_dp_device *dp = dev_get_drvdata(dev);
	const struct rockchip_dp_chip_data *dp_data;
	struct drm_device *drm_dev = data;
	int ret;

	dp_data = of_device_get_match_data(dev);
	if (!dp_data)
		return -ENODEV;

	ret = rockchip_dp_init(dp);
	if (ret < 0)
		return ret;

	dp->data = dp_data;
	dp->drm_dev = drm_dev;

	ret = rockchip_dp_drm_create_encoder(dp);
	if (ret) {
		DRM_ERROR("failed to create drm encoder\n");
		return ret;
	}

	dp->plat_data.encoder = &dp->encoder;

	dp->plat_data.dev_type = ROCKCHIP_DP;
	dp->plat_data.subdev_type = dp_data->chip_type;
	dp->plat_data.power_on_start = rockchip_dp_poweron_start;
	dp->plat_data.power_on_end = rockchip_dp_poweron_end;
	dp->plat_data.power_off = rockchip_dp_powerdown;
	dp->plat_data.get_modes = rockchip_dp_get_modes;

	dp->debugfs_psr = debugfs_create_u64("psr_active_ms", S_IRUGO,
					     drm_dev->primary->debugfs_root,
					     &dp->psr_active_ms);
	if (IS_ERR(dp->debugfs_psr))
		return PTR_ERR(dp->debugfs_psr);

	dp->adp = analogix_dp_bind(dev, dp->drm_dev, &dp->plat_data);
	if (IS_ERR(dp->adp)) {
		debugfs_remove(dp->debugfs_psr);
		return PTR_ERR(dp->adp);
	}

	rockchip_drm_psr_register(&dp->encoder, analogix_dp_psr_set);

	return 0;
}

static void rockchip_dp_unbind(struct device *dev, struct device *master,
			       void *data)
{
	struct rockchip_dp_device *dp = dev_get_drvdata(dev);

	rockchip_drm_psr_unregister(&dp->encoder);

	analogix_dp_unbind(dp->adp);
	debugfs_remove(dp->debugfs_psr);

	dp->adp = ERR_PTR(-ENODEV);
}

static const struct component_ops rockchip_dp_component_ops = {
	.bind = rockchip_dp_bind,
	.unbind = rockchip_dp_unbind,
};

static int rockchip_dp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *panel_node, *port, *endpoint;
	struct drm_panel *panel = NULL;
	struct rockchip_dp_device *dp;

	port = of_graph_get_port_by_id(dev->of_node, 1);
	if (port) {
		endpoint = of_get_child_by_name(port, "endpoint");
		of_node_put(port);
		if (!endpoint) {
			dev_err(dev, "no output endpoint found\n");
			return -EINVAL;
		}

		panel_node = of_graph_get_remote_port_parent(endpoint);
		of_node_put(endpoint);
		if (!panel_node) {
			dev_err(dev, "no output node found\n");
			return -EINVAL;
		}

		panel = of_drm_find_panel(panel_node);
		of_node_put(panel_node);
		if (!panel)
			return -EPROBE_DEFER;
	}

	dp = devm_kzalloc(dev, sizeof(*dp), GFP_KERNEL);
	if (!dp)
		return -ENOMEM;

	dp->dev = dev;
	dp->adp = ERR_PTR(-ENODEV);
	dp->plat_data.panel = panel;

	platform_set_drvdata(pdev, dp);

	return component_add(dev, &rockchip_dp_component_ops);
}

static int rockchip_dp_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &rockchip_dp_component_ops);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int rockchip_dp_suspend(struct device *dev)
{
	struct rockchip_dp_device *dp = dev_get_drvdata(dev);

	if (IS_ERR(dp->adp))
		return 0;

	return analogix_dp_suspend(dp->adp);
}

static int rockchip_dp_resume(struct device *dev)
{
	struct rockchip_dp_device *dp = dev_get_drvdata(dev);

	if (IS_ERR(dp->adp))
		return 0;

	return analogix_dp_resume(dp->adp);
}
#endif

static const struct dev_pm_ops rockchip_dp_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(rockchip_dp_suspend, rockchip_dp_resume)
};

static const struct rockchip_dp_chip_data rk3399_edp = {
	.lcdsel_grf_reg = 0x6250,
	.lcdsel_big = 0 | BIT(21),
	.lcdsel_lit = BIT(5) | BIT(21),
	.chip_type = RK3399_EDP,
};

static const struct rockchip_dp_chip_data rk3288_dp = {
	.lcdsel_grf_reg = 0x025c,
	.lcdsel_big = 0 | BIT(21),
	.lcdsel_lit = BIT(5) | BIT(21),
	.chip_type = RK3288_DP,
};

static const struct of_device_id rockchip_dp_dt_ids[] = {
	{.compatible = "rockchip,rk3288-dp", .data = &rk3288_dp },
	{.compatible = "rockchip,rk3399-edp", .data = &rk3399_edp },
	{}
};
MODULE_DEVICE_TABLE(of, rockchip_dp_dt_ids);

static struct platform_driver rockchip_dp_driver = {
	.probe = rockchip_dp_probe,
	.remove = rockchip_dp_remove,
	.driver = {
		   .name = "rockchip-dp",
		   .owner = THIS_MODULE,
		   .pm = &rockchip_dp_pm_ops,
		   .of_match_table = of_match_ptr(rockchip_dp_dt_ids),
	},
};

module_platform_driver(rockchip_dp_driver);

MODULE_AUTHOR("Yakir Yang <ykk@rock-chips.com>");
MODULE_AUTHOR("Jeff chen <jeff.chen@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip Specific Analogix-DP Driver Extension");
MODULE_LICENSE("GPL v2");
