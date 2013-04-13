/*
 * arch/arm/mach-tegra/cpuquiet.c
 *
 * Cpuquiet driver for Tegra3 CPUs
 *
 * Copyright (c) 2012 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/cpu.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/pm_qos_params.h>
#include <linux/cpuquiet.h>

#include "pm.h"
#include "cpu-tegra.h"
#include "clock.h"
#include "tegra_pmqos.h"

#define CPUQUIET_DEBUG 1
#define CPUQUIET_DEBUG_VERBOSE 0

extern unsigned int best_core_to_turn_up(void);

#define INITIAL_STATE		TEGRA_CPQ_IDLE
#define LP_UP_DELAY_MS_DEF			70
#define LP_DOWN_DELAY_MS_DEF		500

static struct mutex *tegra3_cpu_lock;
static struct workqueue_struct *cpuquiet_wq;
static struct delayed_work cpuquiet_work;
static struct work_struct minmax_work;

static struct kobject *tegra_auto_sysfs_kobject;

static bool is_suspended = false;
static bool no_lp;
static bool enable;
static unsigned int lp_up_delay = LP_UP_DELAY_MS_DEF;
static unsigned int lp_down_delay = LP_DOWN_DELAY_MS_DEF;
static unsigned int idle_top_freq = T3_LP_MAX_FREQ;
static int lp_up_req = 0;

static struct clk *cpu_clk;
static struct clk *cpu_g_clk;
static struct clk *cpu_lp_clk;

static struct cpumask cr_online_requests;
static cputime64_t lp_on_time;
static unsigned int min_cpus = 1;
static unsigned int max_cpus = CONFIG_NR_CPUS;

#define CPUQUIET_TAG                       "[CPUQUIET]: "
/*
 * LPCPU hysteresis default values
 */
#define TEGRA_CPQ_LPCPU_UP_HYS        2
static unsigned int tegra_cpq_lpcpu_up_hys = TEGRA_CPQ_LPCPU_UP_HYS;

enum {
	TEGRA_CPQ_DISABLED = 0,
	TEGRA_CPQ_IDLE,
	TEGRA_CPQ_SWITCH_TO_LP,
	TEGRA_CPQ_SWITCH_TO_G,
};

static inline unsigned int num_cpu_check(unsigned int num)
{
	if (num > CONFIG_NR_CPUS)
		return CONFIG_NR_CPUS;
	if (num < 1)
		return 1;
	return num;
}

unsigned inline int tegra_cpq_max_cpus(void)
{
	unsigned int max_cpus_qos = pm_qos_request(PM_QOS_MAX_ONLINE_CPUS);	
	unsigned int num = min(max_cpus_qos, max_cpus);
	return num_cpu_check(num);
}

unsigned inline int tegra_cpq_min_cpus(void)
{
	unsigned int min_cpus_qos = pm_qos_request(PM_QOS_MIN_ONLINE_CPUS);
	unsigned int num = max(min_cpus_qos, min_cpus);
	return num_cpu_check(num);
}

static inline bool lp_possible(void)
{
	return !is_lp_cluster() && !no_lp && !(tegra_cpq_min_cpus() >= 2) && num_online_cpus() == 1;
}

static inline void show_status(const char* extra, cputime64_t on_time, int cpu)
{
#if CPUQUIET_DEBUG
	if(on_time)
		pr_info(CPUQUIET_TAG "%s Mask=[%d.%d%d%d%d]|lp_on_time=%llu\n",
    		extra, is_lp_cluster(), ((is_lp_cluster() == 1) ? 0 : cpu_online(0)),
        	cpu_online(1), cpu_online(2), cpu_online(3), on_time);
	else		
		if(cpu>0)
			pr_info(CPUQUIET_TAG "%s %d Mask=[%d.%d%d%d%d]\n",
    			extra, cpu, is_lp_cluster(), ((is_lp_cluster() == 1) ? 0 : cpu_online(0)),
        		cpu_online(1), cpu_online(2), cpu_online(3));

		else
			pr_info(CPUQUIET_TAG "%s Mask=[%d.%d%d%d%d]\n",
    			extra, is_lp_cluster(), ((is_lp_cluster() == 1) ? 0 : cpu_online(0)),
        		cpu_online(1), cpu_online(2), cpu_online(3));
#endif
}

static int cpq_state;

static int update_core_config(unsigned int cpunumber, bool up)
{
	int ret = -EINVAL;
	unsigned int nr_cpus = num_online_cpus();
	int max_cpus = tegra_cpq_max_cpus();
	int min_cpus = tegra_cpq_min_cpus();

	if (cpq_state == TEGRA_CPQ_DISABLED || cpunumber >= nr_cpu_ids)
		return ret;

	// if we are in the state of switching to LP mode
	// block any up request else we will end up 
	// in a locked state with > 1 core on and governor inactive
	if (cpq_state == TEGRA_CPQ_SWITCH_TO_LP && up)
		return ret;
		
	if (up) {
		if(is_lp_cluster()) {
			cpumask_set_cpu(cpunumber, &cr_online_requests);
			ret = -EBUSY;
		} else {
			if (nr_cpus < max_cpus){
				show_status("UP", 0, cpunumber);
				ret = cpu_up(cpunumber);
			}
		}
	} else {
		if (is_lp_cluster()) {
			ret = -EBUSY;
		} else {
			if (nr_cpus > 1 && nr_cpus > min_cpus){
				show_status("DOWN", 0, cpunumber);
				ret = cpu_down(cpunumber);
			}
		}
	}
		
	return ret;
}

static int tegra_quiesence_cpu(unsigned int cpunumber)
{
	return update_core_config(cpunumber, false);
}

static int tegra_wake_cpu(unsigned int cpunumber)
{
	return update_core_config(cpunumber, true);
}

static struct cpuquiet_driver tegra_cpuquiet_driver = {
	.name                   = "tegra",
	.quiesence_cpu          = tegra_quiesence_cpu,
	.wake_cpu               = tegra_wake_cpu,
};

static void apply_core_config(void)
{
	unsigned int cpu;

	if (is_lp_cluster() || cpq_state == TEGRA_CPQ_DISABLED)
		return;

	for_each_cpu_mask(cpu, cr_online_requests) {
		if (cpu < nr_cpu_ids && !cpu_online(cpu))
			if (!tegra_wake_cpu(cpu))
				cpumask_clear_cpu(cpu, &cr_online_requests);
	}
}

static void tegra_cpuquiet_work_func(struct work_struct *work)
{
	int device_busy = -1;
	cputime64_t on_time = 0;

	mutex_lock(tegra3_cpu_lock);

	switch(cpq_state) {
		case TEGRA_CPQ_DISABLED:
		case TEGRA_CPQ_IDLE:
			break;
		case TEGRA_CPQ_SWITCH_TO_G:
			if (is_lp_cluster()) {
				/* set rate to max of LP mode */
				clk_set_rate(cpu_clk, T3_LP_MAX_FREQ * 1000);
				if(!clk_set_parent(cpu_clk, cpu_g_clk)) {
					on_time = ktime_to_ms(ktime_get()) - lp_on_time;
					show_status("LP -> off", on_time, -1);
					/*catch-up with governor target speed */
					tegra_cpu_set_speed_cap(NULL);
					/* process pending core requests*/
					device_busy = 0;
				}
			}
#if CPUQUIET_DEBUG_VERBOSE
			else
				pr_info(CPUQUIET_TAG "skipping queued TEGRA_CPQ_SWITCH_TO_G");
#endif
			break;
		case TEGRA_CPQ_SWITCH_TO_LP:
			if (lp_possible()) {
				// this can fail expected!
				// dont switch to LP if freq is too high to not force
				// a slow-down. could be changed from start of down delay
				if (!clk_set_parent(cpu_clk, cpu_lp_clk)) {
					show_status("LP -> on", 0, -1);
					/*catch-up with governor target speed*/
					tegra_cpu_set_speed_cap(NULL);
					device_busy = 1;
					lp_on_time = ktime_to_ms(ktime_get());
				}
#if CPUQUIET_DEBUG_VERBOSE
				else
					pr_info(CPUQUIET_TAG "skipping queued TEGRA_CPQ_SWITCH_TO_LP - clk_set_parent failed");
#endif
			}
#if CPUQUIET_DEBUG_VERBOSE
			else
				pr_info(CPUQUIET_TAG "skipping queued TEGRA_CPQ_SWITCH_TO_LP - cond failed");
#endif			
			break;
		default:
			pr_err(CPUQUIET_TAG "%s: invalid tegra hotplug state %d\n",
		       __func__, cpq_state);
	}

	mutex_unlock(tegra3_cpu_lock);

	if (device_busy == 1) {
		cpuquiet_device_busy();
	} else if (!device_busy) {
		apply_core_config();
		cpuquiet_device_free();
	}
}

static void min_max_constraints_workfunc(struct work_struct *work)
{
	int count = -1;
	bool up = false;
	unsigned int cpu;

	int nr_cpus = num_online_cpus();
	int max_cpus = tegra_cpq_max_cpus();
	int min_cpus = tegra_cpq_min_cpus();
	
	if (cpq_state == TEGRA_CPQ_DISABLED)
		return;

	if (is_lp_cluster())
		return;

	if (nr_cpus < min_cpus) {
		up = true;
		count = min_cpus - nr_cpus;
	} else if (nr_cpus > max_cpus && max_cpus >= min_cpus) {
		count = nr_cpus - max_cpus;
	}

	for (;count > 0; count--) {
		if (up) {
			cpu = best_core_to_turn_up();
			if (cpu < nr_cpu_ids){
				show_status("UP", 0, cpu);
				cpu_up(cpu);
			}
			else
				break;
		} else {
			cpu = cpumask_next(0, cpu_online_mask);
			if (cpu < nr_cpu_ids){
				show_status("DOWN", 0, cpu);
				cpu_down(cpu);
			}
			else
				break;
		}
	}
}

static void min_cpus_change(void)
{
	bool g_cluster = false;
    cputime64_t on_time = 0;
	
	if (cpq_state == TEGRA_CPQ_DISABLED)
		return;

	mutex_lock(tegra3_cpu_lock);

	if ((tegra_cpq_min_cpus() >= 2) && is_lp_cluster()) {
		/* set rate to max of LP mode */
		clk_set_rate(cpu_clk, T3_LP_MAX_FREQ * 1000);
		clk_set_parent(cpu_clk, cpu_g_clk);
		
		on_time = ktime_to_ms(ktime_get()) - lp_on_time;
		show_status("LP -> off - min_cpus_change", on_time, -1);

		g_cluster = true;
	}

	tegra_cpu_set_speed_cap(NULL);
	mutex_unlock(tegra3_cpu_lock);

	schedule_work(&minmax_work);

	if (g_cluster)
		cpuquiet_device_free();
}

static int min_cpus_notify(struct notifier_block *nb, unsigned long n, void *p)
{
	pr_info(CPUQUIET_TAG "PM QoS PM_QOS_MIN_ONLINE_CPUS %lu\n", n);

	if (n < 1 || n > CONFIG_NR_CPUS)
		return NOTIFY_OK;
	
	min_cpus_change();

	return NOTIFY_OK;
}

static void max_cpus_change(void)
{	
	if (cpq_state == TEGRA_CPQ_DISABLED)
		return;

	if (tegra_cpq_max_cpus() < num_online_cpus())
		schedule_work(&minmax_work);
}

static int max_cpus_notify(struct notifier_block *nb, unsigned long n, void *p)
{
	pr_info(CPUQUIET_TAG "PM QoS PM_QOS_MAX_ONLINE_CPUS %lu\n", n);

	if (n < 1)
		return NOTIFY_OK;
	
	max_cpus_change();

	return NOTIFY_OK;
}

int tegra_cpuquiet_force_gmode(void)
{
    cputime64_t on_time = 0;

	if (no_lp)
		return -EBUSY;
		
	if (!is_g_cluster_present())
		return -EBUSY;

	if (cpq_state == TEGRA_CPQ_DISABLED)
		return -EBUSY;

	if (is_lp_cluster()) {
		mutex_lock(tegra3_cpu_lock);

		/* set rate to max of LP mode */
		clk_set_rate(cpu_clk, T3_LP_MAX_FREQ * 1000);
		if (clk_set_parent(cpu_clk, cpu_g_clk)){
			pr_info(CPUQUIET_TAG "tegra_cpuquiet_force_gmode - clk_set_parent failed\n");
    		mutex_unlock(tegra3_cpu_lock);
    		return -EBUSY;
		}
		
		on_time = ktime_to_ms(ktime_get()) - lp_on_time;
		show_status("LP -> off - force", on_time, -1);

        lp_up_req = 0;

    	mutex_unlock(tegra3_cpu_lock);

		cpuquiet_device_free();
	}
	
	return 0;
}

int tegra_cpuquiet_force_gmode_locked(void)
{
    cputime64_t on_time = 0;

	if (no_lp)
		return -EBUSY;
		
	if (!is_g_cluster_present())
		return -EBUSY;

	if (cpq_state == TEGRA_CPQ_DISABLED)
		return -EBUSY;

	if (is_lp_cluster()) {
		/* set rate to max of LP mode */
		clk_set_rate(cpu_clk, T3_LP_MAX_FREQ * 1000);
		if (clk_set_parent(cpu_clk, cpu_g_clk)){
			pr_info(CPUQUIET_TAG "tegra_cpuquiet_force_gmode - clk_set_parent failed\n");
    		return -EBUSY;
		}
		
		on_time = ktime_to_ms(ktime_get()) - lp_on_time;
		show_status("LP -> off - force", on_time, -1);

        lp_up_req = 0;
	}
	return 0;
}

void tegra_cpuquiet_device_free(void)
{
	cpuquiet_device_free();
}

void tegra_auto_hotplug_governor(unsigned int cpu_freq, bool suspend)
{
	if (!is_g_cluster_present())
		return;

	if (cpq_state == TEGRA_CPQ_DISABLED)
		return;

	cpq_state = TEGRA_CPQ_IDLE;
	is_suspended = suspend;
	
	if (suspend) {
        lp_up_req = 0;
		return;
	}

	if (is_lp_cluster() && 
			(cpu_freq > idle_top_freq || no_lp)) {
       	cpq_state = TEGRA_CPQ_SWITCH_TO_G;
		queue_delayed_work(cpuquiet_wq, &cpuquiet_work, msecs_to_jiffies(lp_up_delay));
	} else if (cpu_freq <= idle_top_freq && lp_possible()) {
		lp_up_req++;
        if (lp_up_req > tegra_cpq_lpcpu_up_hys) {
			cpq_state = TEGRA_CPQ_SWITCH_TO_LP;
			lp_up_req = 0;
			queue_delayed_work(cpuquiet_wq, &cpuquiet_work, msecs_to_jiffies(lp_down_delay));
		}
	} else {
        lp_up_req = 0;
	}
}

static struct notifier_block min_cpus_notifier = {
	.notifier_call = min_cpus_notify,
};

static struct notifier_block max_cpus_notifier = {
	.notifier_call = max_cpus_notify,
};

static void enable_callback(struct cpuquiet_attribute *attr)
{
	int disabled = -1;

	mutex_lock(tegra3_cpu_lock);

	if (!enable && cpq_state != TEGRA_CPQ_DISABLED) {
		disabled = 1;
		cpq_state = TEGRA_CPQ_DISABLED;
	} else if (enable && cpq_state == TEGRA_CPQ_DISABLED) {
		disabled = 0;
		cpq_state = TEGRA_CPQ_IDLE;
		tegra_cpu_set_speed_cap(NULL);
	}

	mutex_unlock(tegra3_cpu_lock);

	if (disabled == -1)
		return;

	if (disabled == 1) {
		cancel_delayed_work_sync(&cpuquiet_work);
		pr_info(CPUQUIET_TAG "enable_callback: clusterswitch disabled\n");
		cpuquiet_device_busy();
	} else if (!disabled) {
		pr_info(CPUQUIET_TAG "enable_callback: clusterswitch enabled\n");
		cpuquiet_device_free();
	}
}

ssize_t show_min_cpus(struct cpuquiet_attribute *cattr, char *buf)
{
	char *out = buf;
		
	out += sprintf(out, "%d\n", min_cpus);

	return out - buf;
}

ssize_t store_min_cpus(struct cpuquiet_attribute *cattr,
					const char *buf, size_t count)
{
	int ret;
	unsigned int n;
	
	ret = sscanf(buf, "%d", &n);

	if ((ret != 1) || n < 1 || n > CONFIG_NR_CPUS)
		return -EINVAL;
	
	min_cpus = n;
	min_cpus_change();
	
	return count;
}

ssize_t show_max_cpus(struct cpuquiet_attribute *cattr, char *buf)
{
	char *out = buf;
		
	out += sprintf(out, "%d\n", max_cpus);

	return out - buf;
}

ssize_t store_max_cpus(struct cpuquiet_attribute *cattr,
					const char *buf, size_t count)
{
	int ret;
	unsigned int n;
	
	ret = sscanf(buf, "%d", &n);

	if ((ret != 1) || n < 1 || n > CONFIG_NR_CPUS)
		return -EINVAL;

	max_cpus = n;	
	max_cpus_change();
		
	return count;
}

CPQ_BASIC_ATTRIBUTE(no_lp, 0644, bool);
CPQ_BASIC_ATTRIBUTE(lp_up_delay, 0644, uint);
CPQ_BASIC_ATTRIBUTE(lp_down_delay, 0644, uint);
CPQ_ATTRIBUTE(enable, 0644, bool, enable_callback);
CPQ_ATTRIBUTE_CUSTOM(min_cpus, 0644, show_min_cpus, store_min_cpus);
CPQ_ATTRIBUTE_CUSTOM(max_cpus, 0644, show_max_cpus, store_max_cpus);
CPQ_BASIC_ATTRIBUTE(tegra_cpq_lpcpu_up_hys, 0644, uint);

static struct attribute *tegra_auto_attributes[] = {
	&no_lp_attr.attr,
	&lp_up_delay_attr.attr,
	&lp_down_delay_attr.attr,
	&enable_attr.attr,
	&min_cpus_attr.attr,
	&max_cpus_attr.attr,
	&tegra_cpq_lpcpu_up_hys_attr.attr,
	NULL,
};

static const struct sysfs_ops tegra_auto_sysfs_ops = {
	.show = cpuquiet_auto_sysfs_show,
	.store = cpuquiet_auto_sysfs_store,
};

static struct kobj_type ktype_sysfs = {
	.sysfs_ops = &tegra_auto_sysfs_ops,
	.default_attrs = tegra_auto_attributes,
};

static int tegra_auto_sysfs(void)
{
	int err;

	tegra_auto_sysfs_kobject = kzalloc(sizeof(*tegra_auto_sysfs_kobject),
					GFP_KERNEL);

	if (!tegra_auto_sysfs_kobject)
		return -ENOMEM;

	err = cpuquiet_kobject_init(tegra_auto_sysfs_kobject, &ktype_sysfs,
				"tegra_cpuquiet");

	if (err)
		kfree(tegra_auto_sysfs_kobject);

	return err;
}

int tegra_auto_hotplug_init(struct mutex *cpu_lock)
{
	int err;

	cpu_clk = clk_get_sys(NULL, "cpu");
	cpu_g_clk = clk_get_sys(NULL, "cpu_g");
	cpu_lp_clk = clk_get_sys(NULL, "cpu_lp");

	if (IS_ERR(cpu_clk) || IS_ERR(cpu_g_clk) || IS_ERR(cpu_lp_clk))
		return -ENOENT;

	/*
	 * Not bound to the issuer CPU (=> high-priority), has rescue worker
	 * task, single-threaded, freezable.
	 */
	cpuquiet_wq = alloc_workqueue(
		"cpuquiet", WQ_UNBOUND | WQ_RESCUER | WQ_FREEZABLE, 1);

	if (!cpuquiet_wq)
		return -ENOMEM;

	INIT_DELAYED_WORK(&cpuquiet_work, tegra_cpuquiet_work_func);
	INIT_WORK(&minmax_work, min_max_constraints_workfunc);

	cpumask_clear(&cr_online_requests);
	tegra3_cpu_lock = cpu_lock;

	cpq_state = INITIAL_STATE;
	enable = cpq_state == TEGRA_CPQ_DISABLED ? false : true;

	pr_info(CPUQUIET_TAG "%s: initialized: %s\n", __func__,
		(cpq_state == TEGRA_CPQ_DISABLED) ? "disabled" : "enabled");

	if (pm_qos_add_notifier(PM_QOS_MIN_ONLINE_CPUS, &min_cpus_notifier))
		pr_err(CPUQUIET_TAG "%s: Failed to register min cpus PM QoS notifier\n",
			__func__);
	if (pm_qos_add_notifier(PM_QOS_MAX_ONLINE_CPUS, &max_cpus_notifier))
		pr_err(CPUQUIET_TAG "%s: Failed to register max cpus PM QoS notifier\n",
			__func__);

	err = cpuquiet_register_driver(&tegra_cpuquiet_driver);
	if (err) {
		destroy_workqueue(cpuquiet_wq);
		return err;
	}

	err = tegra_auto_sysfs();
	if (err) {
		cpuquiet_unregister_driver(&tegra_cpuquiet_driver);
		destroy_workqueue(cpuquiet_wq);
	}

	return err;
}

void tegra_auto_hotplug_exit(void)
{
	destroy_workqueue(cpuquiet_wq);
	cpuquiet_unregister_driver(&tegra_cpuquiet_driver);
	kobject_put(tegra_auto_sysfs_kobject);
}
