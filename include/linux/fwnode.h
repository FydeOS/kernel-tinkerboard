/*
 * fwnode.h - Firmware device node object handle type definition.
 *
 * Copyright (C) 2015, Intel Corporation
 * Author: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _LINUX_FWNODE_H_
#define _LINUX_FWNODE_H_

enum fwnode_type {
	FWNODE_INVALID = 0,
	FWNODE_OF,
	FWNODE_ACPI,
	FWNODE_ACPI_DATA,
	FWNODE_PDATA,
	FWNODE_IRQCHIP,
};

struct fwnode_handle {
	enum fwnode_type type;
	struct fwnode_handle *secondary;
};

struct fwnode_handle *fwnode_get_next_child_node(struct fwnode_handle *node,
						 struct fwnode_handle *child);

#define fwnode_for_each_child_node(node, child) \
	for (child = fwnode_get_next_child_node(node, NULL); child; \
	     child = fwnode_get_next_child_node(node, child))

struct fwnode_handle *fwnode_get_named_child_node(struct fwnode_handle *node,
						  const char *childname);

#endif
