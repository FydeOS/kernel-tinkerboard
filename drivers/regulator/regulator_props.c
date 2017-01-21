/*
 * ACPI helpers for regulator framework
 *
 * Copyright (C) 2011 Texas Instruments, Inc
 * Copyright (C) 2016 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/export.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

static const char *const regulator_states[PM_SUSPEND_MAX + 1] = {
	[PM_SUSPEND_MEM]	= "regulator-state-mem",
	[PM_SUSPEND_MAX]	= "regulator-state-disk",
};

/**
 * device_get_regulation_constraints - extract regulation_constraints info
 * @fwnode: fwnode of device requesting for regulation_constraints
 * @init_data: regulator_init_data structure containing regulation_constraints
 * @desc: regulator description
 *
 * Populates regulation_constraints structure by extracting data from device
 * tree or ACPI node.
 */
void device_get_regulation_constraints(struct fwnode_handle *fwnode,
					struct regulator_init_data *init_data,
					const struct regulator_desc *desc)
{
	struct regulation_constraints *constraints = &init_data->constraints;
	int ret, i;
	u32 pval;
	struct regulator_state *suspend_state = NULL;
	struct fwnode_handle *suspend_fwnode;

	fwnode_property_read_string(fwnode, "regulator-name",
					&constraints->name);

	if (!fwnode_property_read_u32(fwnode, "regulator-min-microvolt", &pval))
		constraints->min_uV = pval;

	if (!fwnode_property_read_u32(fwnode, "regulator-max-microvolt", &pval))
		constraints->max_uV = pval;

	/* Voltage change possible? */
	if (constraints->min_uV != constraints->max_uV)
		constraints->valid_ops_mask |= REGULATOR_CHANGE_VOLTAGE;

	/* Do we have a voltage range, if so try to apply it? */
	if (constraints->min_uV && constraints->max_uV)
		constraints->apply_uV = true;

	if (!fwnode_property_read_u32(fwnode, "regulator-microvolt-offset",
					&pval))
		constraints->uV_offset = pval;
	if (!fwnode_property_read_u32(fwnode, "regulator-min-microamp", &pval))
		constraints->min_uA = pval;
	if (!fwnode_property_read_u32(fwnode, "regulator-max-microamp", &pval))
		constraints->max_uA = pval;

	if (!fwnode_property_read_u32(fwnode,
				"regulator-input-current-limit-microamp",
				&pval))
		constraints->ilim_uA = pval;

	/* Current change possible? */
	if (constraints->min_uA != constraints->max_uA)
		constraints->valid_ops_mask |= REGULATOR_CHANGE_CURRENT;

	constraints->boot_on = fwnode_property_read_bool(fwnode,
							"regulator-boot-on");
	constraints->always_on = fwnode_property_read_bool(fwnode,
							"regulator-always-on");
	if (!constraints->always_on) /* status change should be possible. */
		constraints->valid_ops_mask |= REGULATOR_CHANGE_STATUS;

	constraints->pull_down = fwnode_property_read_bool(fwnode,
							"regulator-pull-down");

	if (fwnode_property_read_bool(fwnode, "regulator-allow-bypass"))
		constraints->valid_ops_mask |= REGULATOR_CHANGE_BYPASS;

	if (fwnode_property_read_bool(fwnode, "regulator-allow-set-load"))
		constraints->valid_ops_mask |= REGULATOR_CHANGE_DRMS;

	ret = fwnode_property_read_u32(fwnode, "regulator-ramp-delay", &pval);
	if (!ret) {
		if (pval)
			constraints->ramp_delay = pval;
		else
			constraints->ramp_disable = true;
	}

	ret = fwnode_property_read_u32(fwnode, "regulator-enable-ramp-delay",
					&pval);
	if (!ret)
		constraints->enable_time = pval;

	constraints->soft_start = fwnode_property_read_bool(fwnode,
							"regulator-soft-start");
	ret = fwnode_property_read_u32(fwnode, "regulator-active-discharge",
					&pval);
	if (!ret) {
		constraints->active_discharge =
				(pval) ? REGULATOR_ACTIVE_DISCHARGE_ENABLE :
					REGULATOR_ACTIVE_DISCHARGE_DISABLE;
	}

	if (!fwnode_property_read_u32(fwnode, "regulator-initial-mode",
					&pval)) {
		if (desc && desc->map_mode) {
			ret = desc->map_mode(pval);
			if (ret == -EINVAL)
				pr_err("invalid mode %u\n", pval);
			else
				constraints->initial_mode = ret;
		} else {
			pr_warn("mapping for mode %d not defined\n", pval);
		}
	}

	if (!fwnode_property_read_u32(fwnode, "regulator-system-load", &pval))
		constraints->system_load = pval;

	constraints->over_current_protection = fwnode_property_read_bool(fwnode,
					"regulator-over-current-protection");

	for (i = 0; i < ARRAY_SIZE(regulator_states); i++) {
		switch (i) {
		case PM_SUSPEND_MEM:
			suspend_state = &constraints->state_mem;
			break;
		case PM_SUSPEND_MAX:
			suspend_state = &constraints->state_disk;
			break;
		case PM_SUSPEND_ON:
		case PM_SUSPEND_FREEZE:
		case PM_SUSPEND_STANDBY:
		default:
			continue;
		}

		suspend_fwnode = fwnode_get_named_child_node(fwnode,
							regulator_states[i]);
		if (!suspend_fwnode || !suspend_state)
			continue;

		if (!fwnode_property_read_u32(suspend_fwnode, "regulator-mode",
						&pval)) {
			if (desc && desc->map_mode) {
				ret = desc->map_mode(pval);
				if (ret == -EINVAL)
					pr_err("invalid mode %u\n", pval);
				else
					suspend_state->mode = ret;
			} else {
				pr_warn("mapping for mode %d not defined\n",
					pval);
			}
		}

		if (fwnode_property_read_bool(suspend_fwnode,
					"regulator-on-in-suspend"))
			suspend_state->enabled = true;
		else if (fwnode_property_read_bool(suspend_fwnode,
						"regulator-off-in-suspend"))
			suspend_state->disabled = true;

		if (!fwnode_property_read_u32(suspend_fwnode,
					"regulator-suspend-microvolt", &pval))
			suspend_state->uV = pval;

		fwnode_handle_put(suspend_fwnode);
		suspend_state = NULL;
		suspend_fwnode = NULL;
	}
}
EXPORT_SYMBOL_GPL(device_get_regulation_constraints);
