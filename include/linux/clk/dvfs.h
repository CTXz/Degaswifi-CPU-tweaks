/*
 *  linux/include/linux/clk/dvfs.h
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef _PXA_DVFS_H_
#define _PXA_DVFS_H_

#include <linux/clk-private.h>
#include <linux/debugfs.h>
/*
 * dvfs_relationship between to rails, "from" and "to"
 * when the rail changes, it will call dvfs_rail_update on the rails
 * in the relationship_to list.
 * when determining the voltage to set a rail to, it will consider each
 * rail in the relationship_from list.
 */
struct dvfs_relationship {
	struct dvfs_rail *to;
	struct dvfs_rail *from;
	int (*solve)(struct dvfs_rail *, struct dvfs_rail *);

	struct list_head to_node; /* node in relationship_to list */
	struct list_head from_node; /* node in relationship_from list */
};

struct dvfs_rail {
	const char *reg_id;
	int min_millivolts;
	int max_millivolts;
	int nominal_millivolts;
	int step;

	struct list_head node;  /* node in dvfs_rail_list */
	struct list_head dvfs;  /* list head of attached dvfs clocks */
	struct list_head relationships_to;
	struct list_head relationships_from;
	struct regulator *reg;
	int millivolts;
	int new_millivolts;

	/* is it must for rails to update in atomic */
	int update_inatomic;

	/* private method for voltage setting, unit mV */
	int (*set_volt)(struct dvfs_rail *rail, int new_millivolts);
};

struct vol_table {
	unsigned long freq; /* in KHz */
	int millivolts;
};

struct dvfs {
	const char *clk_name;

	/* Must be initialized before dvfs_init */
	struct vol_table *vol_freq_table;
	int num_freqs;
	struct dvfs_rail *dvfs_rail;
	unsigned long cur_rate;

	int millivolts;
	struct list_head dvfs_node;
	struct notifier_block nb;
};

int enable_dvfs_on_clk(struct clk *c, struct dvfs *d);
int dvfs_init_rails(struct dvfs_rail *dvfs_rails[], int n);
void dvfs_add_relationships(struct dvfs_relationship *rels, int n);
void dvfs_remove_relationship(struct dvfs_relationship *rel);
int __init dvfs_debugfs_init(struct dentry *clk_debugfs_root);

#endif
