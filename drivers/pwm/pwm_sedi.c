/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <drivers/pwm.h>
#include <device.h>
#include <kernel.h>
#include <init.h>
#include <pm/pm.h>
#include <sys/util.h>
#include "driver/sedi_driver_pwm.h"
#include "driver/sedi_driver_pm.h"

#define DT_DRV_COMPAT intel_pse_pwm

#define MIN_LOW_PERIOD  (1)
#define GET_PWM_MUTEX(dev) (&((struct pwm_runtime *)(dev->data))->pwm_mutex)

struct pwm_runtime {
	uint32_t pin_busy;
	struct k_mutex pwm_mutex;
#ifdef CONFIG_PM_DEVICE
	uint32_t device_power_state;
#endif

};

static int pwm_sedi_pin_stop(const struct device *dev, uint32_t pwm)
{
	uint32_t instance = (uint32_t)(dev->config);
	struct pwm_runtime *rt = dev->data;

	__ASSERT(pwm < SEDI_PWM_ID_NUM, "");

	sedi_pwm_stop(instance, pwm);
	rt->pin_busy &= ~BIT(pwm);
	if (!rt->pin_busy) {
		pm_device_busy_clear(dev);
	}
	return 0;
}

static int pwm_sedi_pin_start(const struct device *dev, uint32_t pwm)
{
	uint32_t instance = (uint32_t)(dev->config);
	struct pwm_runtime *rt = dev->data;

	__ASSERT(pwm < SEDI_PWM_ID_NUM, "");

	rt->pin_busy |= BIT(pwm);
	pm_device_busy_set(dev);
	sedi_pwm_start(instance, pwm);
	return 0;
}

static int pwm_sedi_pin_set(const struct device *dev, uint32_t pwm,
			    uint32_t period_cycles, uint32_t pulse_cycles,
			    pwm_flags_t flags)
{
	ARG_UNUSED(flags);
	uint32_t ret, high, low, max = ~0;
	sedi_pwm_config_t cfg;
	uint32_t instance = (uint32_t)(dev->config);

	__ASSERT(pwm < SEDI_PWM_ID_NUM, "");


	if ((period_cycles == 0) ||
	    (pulse_cycles > period_cycles) ||
	    (period_cycles > max) || (pulse_cycles > max)) {
		return -EINVAL;
	}

	if (pulse_cycles == 0) {
		pwm_sedi_pin_stop(dev, pwm);
		return 0;
	}

	high = pulse_cycles;
	low = period_cycles - pulse_cycles;

	/*
	 * low must be more than zero. Otherwise, the PWM pin will be
	 * turned off. Let's make sure low is always more than zero.
	 */
	if (low == 0) {
		high--;
		low = MIN_LOW_PERIOD;
	}

	cfg.mode = SEDI_PWM_MODE_PWM;
	cfg.intr_enable = false;
	cfg.hi_count = high;
	cfg.lo_count = low;

	k_mutex_lock(GET_PWM_MUTEX(dev), K_FOREVER);

	pwm_sedi_pin_stop(dev, pwm);
	ret = sedi_pwm_set_config(instance, pwm, cfg);
	if (!ret) {
		pwm_sedi_pin_start(dev, pwm);
	}
	k_mutex_unlock(GET_PWM_MUTEX(dev));
	return ret;
}

static int pwm_sedi_get_cycles_per_sec(const struct device *dev, uint32_t pwm,
				       uint64_t *cycles)
{
	if (cycles == NULL) {
		return -EINVAL;
	}

	*cycles = sedi_pm_get_lbw_clock();

	return 0;
}

static const struct pwm_driver_api api_funcs = {
	.pin_set = pwm_sedi_pin_set,
	.get_cycles_per_sec = pwm_sedi_get_cycles_per_sec,
};

static int pwm_init(const struct device *dev)
{
	uint32_t instance = (uint32_t)(dev->config);
	int ret;

	k_mutex_init(GET_PWM_MUTEX(dev));
	ret = sedi_pwm_init(instance, NULL, (void *)dev);
	if (ret != SEDI_DRIVER_OK) {
		return -ENODEV;
	}

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int pwm_suspend_device(const struct device *dev)
{
	uint32_t status = SEDI_DRIVER_OK;
	uint32_t instance = (uint32_t)(dev->config);

	if (pm_device_is_busy(dev)) {
		return -EBUSY;
	}

	status = sedi_pwm_set_power(instance, SEDI_POWER_SUSPEND);
	if (status != SEDI_DRIVER_OK) {
		return -EIO;
	}

	return 0;
}

static int pwm_set_device_low_power(const struct device *dev)
{
	uint32_t status= SEDI_DRIVER_OK;
	uint32_t instance = (uint32_t)(dev->config);

	if (pm_device_is_busy(dev)) {
		return -EBUSY;
	}

	status = sedi_pwm_set_power(instance, SEDI_POWER_LOW);
	if (status != SEDI_DRIVER_OK) {
		return -EIO;
	}
	return 0;
}
static int pwm_resume_device_from_suspend(const struct device *dev)
{
	uint32_t status= SEDI_DRIVER_OK;
	uint32_t instance = (uint32_t)(dev->config);

	status = sedi_pwm_set_power(instance, SEDI_POWER_FULL);
	if (status != SEDI_DRIVER_OK) {
		return -EIO;
	}

	return 0;
}


static int pwm_force_suspend_device(const struct device *dev)
{
	uint32_t status;
	uint32_t instance = (uint32_t)(dev->config);

	status = sedi_pwm_set_power(instance, SEDI_POWER_FORCE_SUSPEND);

	return 0;
}

static int pwm_device_action_cb(const struct device *dev, enum pm_device_action action)
{
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		ret = pwm_suspend_device(dev);
		break;
	case PM_DEVICE_ACTION_RESUME:
		ret = pwm_resume_device_from_suspend(dev);
		break;
	case PM_DEVICE_ACTION_LOW_POWER:
			ret = pwm_set_device_low_power(dev);
			break;

	case PM_DEVICE_ACTION_FORCE_SUSPEND:
		pwm_force_suspend_device(dev);
		break;
	default:
		ret = ENOTSUP;

	}


	return ret;
}
#endif

#define PWM_SEDI_DEVICE_INIT(n)						   \
	static struct pwm_runtime pwm_runtime_##n;			   \
	DEVICE_DEFINE(pwm_##n, DT_INST_LABEL(n),			   \
		      &pwm_init, pwm_device_action_cb, &pwm_runtime_##n,	   \
		      (void *)DT_INST_PROP(n, peripheral_id), POST_KERNEL, \
		      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api_funcs);

DT_INST_FOREACH_STATUS_OKAY(PWM_SEDI_DEVICE_INIT)
