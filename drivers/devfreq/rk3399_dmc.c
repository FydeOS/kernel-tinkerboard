/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd.
 * Author: Lin Huang <hl@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/devfreq-event.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
#include <linux/rwsem.h>
#include <linux/suspend.h>

#include <soc/rockchip/rockchip_sip.h>
#include <soc/rockchip/rk3399_dmc.h>

#include "rk3399_dmc_priv.h"

#define NS_TO_CYCLE(NS, MHz)		((NS * MHz) / NSEC_PER_USEC)

static int rk3399_dmcfreq_target(struct device *dev, unsigned long *freq,
				 u32 flags)
{
	struct rk3399_dmcfreq *dmcfreq = dev_get_drvdata(dev);
	unsigned long old_clk_rate = dmcfreq->rate;
	unsigned long target_volt, target_rate;
	struct arm_smccc_res res;
	struct dev_pm_opp *opp;
	int dram_flag, err, odt_pd_arg0, odt_pd_arg1;
	unsigned int pd_idle_cycle, standby_idle_cycle, sr_idle_cycle;
	unsigned int sr_mc_gate_idle_cycle, srpd_lite_idle_cycle;
	unsigned int ddrcon_mhz;

	rcu_read_lock();
	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp)) {
		rcu_read_unlock();
		return PTR_ERR(opp);
	}

	target_rate = dev_pm_opp_get_freq(opp);
	target_volt = dev_pm_opp_get_voltage(opp);
	rcu_read_unlock();

	if (dmcfreq->rate == target_rate)
		return 0;

	mutex_lock(&dmcfreq->lock);

	dram_flag = 0;
	if (target_rate >= dmcfreq->odt_dis_freq)
		dram_flag = 1;

	/*
	 * idle parameter base on the ddr controller clock which
	 * is half of the ddr frequency.
	 * pd_idle, standby_idle base on the controller clock cycle.
	 * sr_idle_cycle, sr_mc_gate_idle_cycle, srpd_lite_idle_cycle,
	 * base on the 1024 controller clock cycle
	 */
	ddrcon_mhz = target_rate / USEC_PER_SEC / 2;
	pd_idle_cycle = NS_TO_CYCLE(dmcfreq->pd_idle, ddrcon_mhz);
	standby_idle_cycle = NS_TO_CYCLE(dmcfreq->standby_idle, ddrcon_mhz);
	sr_idle_cycle = DIV_ROUND_UP(NS_TO_CYCLE(dmcfreq->sr_idle, ddrcon_mhz),
				     1024);
	sr_mc_gate_idle_cycle = DIV_ROUND_UP(
			NS_TO_CYCLE(dmcfreq->sr_mc_gate_idle, ddrcon_mhz),
				    1024);
	srpd_lite_idle_cycle = DIV_ROUND_UP(NS_TO_CYCLE(dmcfreq->srpd_lite_idle,
							ddrcon_mhz), 1024);

	/*
	 * odt_pd_arg0:
	 * bit0-7: sr_idle value
	 * bit8-15: sr_mc_gate_idle
	 * bit16-31: standby_idle
	 */
	odt_pd_arg0 = (sr_idle_cycle & 0xff) |
		      ((sr_mc_gate_idle_cycle & 0xff) << 8) |
		      ((standby_idle_cycle & 0xffff) << 16);

	/* odt_pd_arg1:
	 * bit0-11: pd_idle
	 * bit16-27: srpd_lite_idle
	 */
	odt_pd_arg1 = (pd_idle_cycle & 0xfff) |
		      ((srpd_lite_idle_cycle & 0xfff) << 16);

	if (target_rate >= dmcfreq->sr_idle_dis_freq)
		odt_pd_arg0 = odt_pd_arg0 & 0xffffff00;
	if (target_rate >= dmcfreq->sr_mc_gate_idle_dis_freq)
		odt_pd_arg0 = odt_pd_arg0 & 0xffff00ff;
	if (target_rate >= dmcfreq->standby_idle_dis_freq)
		odt_pd_arg0 = odt_pd_arg0 & 0x0000ffff;

	if (target_rate >= dmcfreq->pd_idle_dis_freq)
		odt_pd_arg1 = odt_pd_arg1 & 0xfffff000;
	if (target_rate >= dmcfreq->srpd_lite_idle_dis_freq)
		odt_pd_arg1 = odt_pd_arg1 & 0xf000ffff;

	arm_smccc_smc(ROCKCHIP_SIP_DRAM_FREQ, odt_pd_arg0, odt_pd_arg1,
		      ROCKCHIP_SIP_CONFIG_DRAM_SET_ODT_PD,
		      dram_flag, 0, 0, 0, &res);

	/*
	 * If frequency scaling from low to high, adjust voltage first.
	 * If frequency scaling from high to low, adjust frequency first.
	 */
	if (old_clk_rate < target_rate) {
		err = regulator_set_voltage(dmcfreq->vdd_center, target_volt,
					    target_volt);
		if (err) {
			dev_err(dev, "Cannot set voltage %lu uV\n",
				target_volt);
			goto out;
		}
	}

	err = clk_set_rate(dmcfreq->dmc_clk, target_rate);
	if (err) {
		dev_err(dev, "Cannot set frequency %lu (%d)\n", target_rate,
			err);
		regulator_set_voltage(dmcfreq->vdd_center, dmcfreq->volt,
				      dmcfreq->volt);
		goto out;
	}

	/*
	 * Setting the dpll is asynchronous since clk_set_rate grabs a global
	 * common clk lock and set_rate for the dpll takes up to one display
	 * frame to complete. We still need to wait for the set_rate to complete
	 * here, though, before we change voltage.
	 */
	rockchip_ddrclk_wait_set_rate(dmcfreq->dmc_clk);
	/*
	 * Check the dpll rate,
	 * There only two result we will get,
	 * 1. Ddr frequency scaling fail, we still get the old rate.
	 * 2. Ddr frequency scaling sucessful, we get the rate we set.
	 */
	dmcfreq->rate = clk_get_rate(dmcfreq->dmc_clk);

	/* If get the incorrect rate, set voltage to old value. */
	if (dmcfreq->rate != target_rate) {
		dev_dbg(dev, "Got wrong frequency, Request %lu, Current %lu\n",
			target_rate, dmcfreq->rate);
		regulator_set_voltage(dmcfreq->vdd_center, dmcfreq->volt,
				      dmcfreq->volt);
		goto out;
	} else if (old_clk_rate > target_rate)
		err = regulator_set_voltage(dmcfreq->vdd_center, target_volt,
					    target_volt);
	if (err)
		dev_err(dev, "Cannot set voltage %lu uV\n", target_volt);
	else
		dmcfreq->volt = target_volt;

out:
	mutex_unlock(&dmcfreq->lock);
	return err;
}

static int rk3399_dmcfreq_get_dev_status(struct device *dev,
					 struct devfreq_dev_status *stat)
{
	struct rk3399_dmcfreq *dmcfreq = dev_get_drvdata(dev);
	struct devfreq_event_data edata;
	int ret = 0;

	ret = devfreq_event_get_event(dmcfreq->edev, &edata);
	if (ret < 0)
		return ret;

	stat->current_frequency = dmcfreq->rate;
	stat->busy_time = edata.load_count;
	stat->total_time = edata.total_count;

	return ret;
}

static int rk3399_dmcfreq_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct rk3399_dmcfreq *dmcfreq = dev_get_drvdata(dev);

	*freq = dmcfreq->rate;

	return 0;
}

static struct devfreq_dev_profile rk3399_devfreq_dmc_profile = {
	.polling_ms	= 200,
	.target		= rk3399_dmcfreq_target,
	.get_dev_status	= rk3399_dmcfreq_get_dev_status,
	.get_cur_freq	= rk3399_dmcfreq_get_cur_freq,
};

static __maybe_unused int rk3399_dmcfreq_suspend(struct device *dev)
{
	struct rk3399_dmcfreq *dmcfreq = dev_get_drvdata(dev);
	int ret = 0;

	ret = devfreq_event_disable_edev(dmcfreq->edev);
	if (ret < 0) {
		dev_err(dev, "failed to disable the devfreq-event devices\n");
		return ret;
	}

	ret = devfreq_suspend_device(dmcfreq->devfreq);
	if (ret < 0) {
		dev_err(dev, "failed to suspend the devfreq devices\n");
		return ret;
	}

	return 0;
}

static __maybe_unused int rk3399_dmcfreq_resume(struct device *dev)
{
	struct rk3399_dmcfreq *dmcfreq = dev_get_drvdata(dev);
	int ret = 0;

	ret = devfreq_event_enable_edev(dmcfreq->edev);
	if (ret < 0) {
		dev_err(dev, "failed to enable the devfreq-event devices\n");
		return ret;
	}

	ret = devfreq_resume_device(dmcfreq->devfreq);
	if (ret < 0) {
		dev_err(dev, "failed to resume the devfreq devices\n");
		return ret;
	}
	return ret;
}

static SIMPLE_DEV_PM_OPS(rk3399_dmcfreq_pm, rk3399_dmcfreq_suspend,
			 rk3399_dmcfreq_resume);

int rockchip_dmcfreq_register_clk_sync_nb(struct devfreq *devfreq,
					struct notifier_block *nb)
{
	struct rk3399_dmcfreq *dmcfreq = dev_get_drvdata(devfreq->dev.parent);
	int ret;

	mutex_lock(&dmcfreq->en_lock);
	/*
	 * We have a short amount of time (~1ms or less typically) to run
	 * dmcfreq after we sync with the notifier, so syncing with more than
	 * one notifier is not generally possible. Thus, if more than one sync
	 * notifier is registered, disable dmcfreq.
	 */
	if (dmcfreq->num_sync_nb == 1 && dmcfreq->disable_count <= 0) {
		rockchip_ddrclk_set_timeout_en(dmcfreq->dmc_clk, false);
		devfreq_suspend_device(devfreq);
	}

	ret = rockchip_ddrclk_register_sync_nb(dmcfreq->dmc_clk, nb);
	if (ret == 0)
		dmcfreq->num_sync_nb++;
	else if (dmcfreq->num_sync_nb == 1 && dmcfreq->disable_count <= 0) {
		rockchip_ddrclk_set_timeout_en(dmcfreq->dmc_clk, true);
		devfreq_resume_device(devfreq);
	}

	mutex_unlock(&dmcfreq->en_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(rockchip_dmcfreq_register_clk_sync_nb);

int rockchip_dmcfreq_unregister_clk_sync_nb(struct devfreq *devfreq,
					  struct notifier_block *nb)
{
	struct rk3399_dmcfreq *dmcfreq = dev_get_drvdata(devfreq->dev.parent);
	int ret;

	mutex_lock(&dmcfreq->en_lock);
	ret = rockchip_ddrclk_unregister_sync_nb(dmcfreq->dmc_clk, nb);
	if (ret == 0) {
		dmcfreq->num_sync_nb--;
		if (dmcfreq->num_sync_nb == 1 && dmcfreq->disable_count <= 0) {
			rockchip_ddrclk_set_timeout_en(dmcfreq->dmc_clk, true);
			devfreq_resume_device(devfreq);
		}
	}

	mutex_unlock(&dmcfreq->en_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(rockchip_dmcfreq_unregister_clk_sync_nb);

int rockchip_dmcfreq_block(struct devfreq *devfreq)
{
	struct rk3399_dmcfreq *dmcfreq = dev_get_drvdata(devfreq->dev.parent);

	mutex_lock(&dmcfreq->en_lock);
	if (dmcfreq->num_sync_nb <= 1 && dmcfreq->disable_count <= 0) {
		rockchip_ddrclk_set_timeout_en(dmcfreq->dmc_clk, false);
		devfreq_suspend_device(devfreq);
	}

	dmcfreq->disable_count++;
	mutex_unlock(&dmcfreq->en_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(rockchip_dmcfreq_block);

int rockchip_dmcfreq_unblock(struct devfreq *devfreq)
{
	struct rk3399_dmcfreq *dmcfreq = dev_get_drvdata(devfreq->dev.parent);

	mutex_lock(&dmcfreq->en_lock);
	dmcfreq->disable_count--;
	if (dmcfreq->num_sync_nb <= 1 && dmcfreq->disable_count <= 0) {
		rockchip_ddrclk_set_timeout_en(dmcfreq->dmc_clk, true);
		devfreq_resume_device(devfreq);
	}

	WARN_ON(dmcfreq->disable_count < 0);
	mutex_unlock(&dmcfreq->en_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(rockchip_dmcfreq_unblock);

static int rk3399_dmcfreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct rk3399_dmcfreq *data;
	int ret;
	struct dev_pm_opp *opp;

	data = devm_kzalloc(dev, sizeof(struct rk3399_dmcfreq), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	mutex_init(&data->lock);
	mutex_init(&data->en_lock);

	data->vdd_center = devm_regulator_get(dev, "center");
	if (IS_ERR(data->vdd_center)) {
		dev_err(dev, "Cannot get the regulator \"center\"\n");
		return PTR_ERR(data->vdd_center);
	}

	data->dmc_clk = devm_clk_get(dev, "dmc_clk");
	if (IS_ERR(data->dmc_clk)) {
		dev_err(dev, "Cannot get the clk dmc_clk\n");
		return PTR_ERR(data->dmc_clk);
	};

	data->edev = devfreq_event_get_edev_by_phandle(dev, 0);
	if (IS_ERR(data->edev))
		return -EPROBE_DEFER;

	ret = devfreq_event_enable_edev(data->edev);
	if (ret < 0) {
		dev_err(dev, "failed to enable devfreq-event devices\n");
		return ret;
	}

	of_property_read_u32(np, "rockchip,pd_idle_ns", &data->pd_idle);
	of_property_read_u32(np, "rockchip,sr_idle_ns", &data->sr_idle);
	of_property_read_u32(np, "rockchip,sr_mc_gate_idle_ns",
			     &data->sr_mc_gate_idle);
	of_property_read_u32(np, "rockchip,srpd_lite_idle_ns",
			     &data->srpd_lite_idle);
	of_property_read_u32(np, "rockchip,standby_idle_ns",
			     &data->standby_idle);
	of_property_read_u32(np, "rockchip,pd_idle_dis_freq",
			     &data->pd_idle_dis_freq);
	of_property_read_u32(np, "rockchip,sr_idle_dis_freq",
			     &data->sr_idle_dis_freq);
	of_property_read_u32(np, "rockchip,sr_mc_gate_idle_dis_freq",
			     &data->sr_mc_gate_idle_dis_freq);
	of_property_read_u32(np, "rockchip,srpd_lite_idle_dis_freq",
			     &data->srpd_lite_idle_dis_freq);
	of_property_read_u32(np, "rockchip,standby_idle_dis_freq",
			     &data->standby_idle_dis_freq);
	of_property_read_u32(np, "rockchip,odt_dis_freq", &data->odt_dis_freq);

	/*
	 * We add a devfreq driver to our parent since it has a device tree node
	 * with operating points.
	 */
	if (dev_pm_opp_of_add_table(dev)) {
		dev_err(dev, "Invalid operating-points in device tree.\n");
		rcu_read_unlock();
		return -EINVAL;
	}

	of_property_read_u32(np, "upthreshold",
			     &data->ondemand_data.upthreshold);
	of_property_read_u32(np, "downdifferential",
			     &data->ondemand_data.downdifferential);

	data->rate = clk_get_rate(data->dmc_clk);

	rcu_read_lock();
	opp = devfreq_recommended_opp(dev, &data->rate, 0);
	if (IS_ERR(opp)) {
		rcu_read_unlock();
		return PTR_ERR(opp);
	}
	data->volt = dev_pm_opp_get_voltage(opp);
	rcu_read_unlock();

	rk3399_devfreq_dmc_profile.initial_freq = data->rate;

	data->devfreq = devfreq_add_device(dev,
					   &rk3399_devfreq_dmc_profile,
					   "simple_ondemand",
					   &data->ondemand_data);
	if (IS_ERR(data->devfreq))
		return PTR_ERR(data->devfreq);
	devm_devfreq_register_opp_notifier(dev, data->devfreq);

	data->dev = dev;
	platform_set_drvdata(pdev, data);
	pd_register_notify_to_dmc(data->devfreq);

	return 0;
}

static int rk3399_dmcfreq_remove(struct platform_device *pdev)
{
	struct rk3399_dmcfreq *dmcfreq = platform_get_drvdata(pdev);

	regulator_put(dmcfreq->vdd_center);

	return 0;
}

static const struct of_device_id rk3399dmc_devfreq_of_match[] = {
	{ .compatible = "rockchip,rk3399-dmc" },
	{ },
};

static struct platform_driver rk3399_dmcfreq_driver = {
	.probe	= rk3399_dmcfreq_probe,
	.remove	= rk3399_dmcfreq_remove,
	.driver = {
		.name	= "rk3399-dmc-freq",
		.pm	= &rk3399_dmcfreq_pm,
		.of_match_table = rk3399dmc_devfreq_of_match,
	},
};
module_platform_driver(rk3399_dmcfreq_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Lin Huang <hl@rock-chips.com>");
MODULE_DESCRIPTION("RK3399 dmcfreq driver with devfreq framework");
