/*
 * arch/arm/mach-tegra/htc_perf.c
 *
 * Copyright (C) 2012 HTC Corporation.
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

#include <linux/cpufreq.h>
#include <linux/kobject.h>
#include <linux/mutex.h>
#include <linux/pm_qos_params.h>
#include <linux/workqueue.h>
#include <linux/nct1008.h> /* for thermal temperature */
#include "cpu-tegra.h"
#include "fuse.h"

#define htc_perf_attr(attrbute) 				\
static struct kobj_attribute attrbute##_attr = {	\
	.attr	= {					\
		.name = __stringify(attrbute),		\
		.mode = 0644,				\
	},						\
	.show	= attrbute##_show,			\
	.store	= attrbute##_store,			\
}

#define FUSE_CPUIDDQ 0x118

struct kobject *htc_perf_kobj;

static ssize_t cpuiddq_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	u32 reg;
        reg = tegra_fuse_readl(FUSE_CPUIDDQ);
        return sprintf(buf, "0x%x\n", reg);
}

static ssize_t cpuiddq_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	pr_info("[htc_perf] cpuiddq do nothing");
	return 0;
}
htc_perf_attr(cpuiddq);

static ssize_t cpu_temp_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	long temperature;
	struct nct1008_data *thermal_data = get_pwr_data();
	nct1008_thermal_get_temp(thermal_data, &temperature);
	temperature /= 10;
	return sprintf(buf, "%d.%d\n", (int)temperature/100, (int)temperature%100);
}

static ssize_t cpu_temp_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	pr_info("[htc_perf] camera temperature do nothing");
	return 0;
}

htc_perf_attr(cpu_temp);

static unsigned int cpu_debug_on = 0;

static ssize_t cpu_debug_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", cpu_debug_on);
}

static ssize_t cpu_debug_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;

	sscanf(buf, "%u", &value);
	cpu_debug_on = value;

	return count;
}

htc_perf_attr(cpu_debug);

unsigned int get_cpu_debug(void)
{
	return cpu_debug_on;
}
EXPORT_SYMBOL(get_cpu_debug);

static struct attribute * g[] = {
	&cpu_temp_attr.attr,
	&cpu_debug_attr.attr,
	&cpuiddq_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

static int __init htc_perf_init(void)
{
	pr_info("[htc_perf] htc_perf_init\n");
        htc_perf_kobj = kobject_create_and_add("htc", NULL);

    if (!htc_perf_kobj)
		return -ENOMEM;

	return sysfs_create_group(htc_perf_kobj, &attr_group);
}
late_initcall(htc_perf_init);
