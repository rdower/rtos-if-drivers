/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <device.h>
#include <init.h>
#include <drivers/counter.h>
#include "driver/sedi_driver_pwm.h"
#include "driver/sedi_driver_pm.h"

#define DT_DRV_COMPAT intel_pse_pwm

#define COUNTER_BLOCK_0         0
#define COUNTER_BLOCK_1         1
#define GET_COUNTER_MUTEX(dev) (&((struct block_runtime *) \
				  (dev->data))->counter_mutex)

#define COUNTER_SEDI_BLOCK_INIT(n)				       \
	static void irq_config_blk_##n(const struct device *dev)       \
	{							       \
		ARG_UNUSED(dev);				       \
		IRQ_CONNECT(DT_INST_IRQN(n),			       \
			    DT_INST_IRQ(n, priority),		       \
			    sedi_pwm_isr_handler,		       \
			    DT_INST_PROP(n, peripheral_id), 0);	       \
		irq_enable(DT_INST_IRQN(n));			       \
	}							       \
	static const struct block_config config_info_##n = {	       \
		.irq_config_func = irq_config_blk_##n,		       \
	};							       \
/* Internal block device*/					       \
	DEVICE_DEFINE(counter_block_##n, "counter_block_##n",	       \
		      counter_block_init, NULL,			       \
		      &block_rt[DT_INST_PROP(n, peripheral_id)],       \
		      &config_info_##n, POST_KERNEL,		       \
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

#define GET_BLOCK_INSTANCE(counter_dev)	\
	(((struct block_runtime *)	\
	  counter_dev->data)->instance)

#define GET_COUNTER_INSTANCE(counter_dev) \
	(((struct sedi_counter_cfg *)	  \
	  counter_dev->config)->instance)

#define COUNTER_DEVICE_INIT(num, prio)			   \
	static struct					   \
	sedi_counter_cfg counter_cfg_##num = {		   \
		.info = {				   \
			.max_top_value = UINT32_MAX,	   \
			.freq = 1U,			   \
			.flags = 0U,			   \
			.channels = 1U,			   \
		},					   \
		.instance = (num % SEDI_PWM_ID_NUM),	   \
	};						   \
	DEVICE_DEFINE(counter_##num, "COUNTER_" # num,	   \
		      counter_init, NULL,		   \
		      &block_rt[num / SEDI_PWM_ID_NUM],	   \
		      &counter_cfg_##num,		   \
		      POST_KERNEL, prio, &counter_pwm_sedi_api)

struct block_config {
	void (*irq_config_func)(const struct device *dev);
};

struct sedi_counter_cfg {
	struct counter_config_info info;
	uint32_t instance;
};

struct block_runtime {
	uint32_t instance;
	uint32_t repeat;
	uint32_t irq;
	struct k_mutex counter_mutex;
	const struct device *counter_dev[SEDI_PWM_ID_NUM];
	counter_top_callback_t cb[SEDI_PWM_ID_NUM];
	void *cb_data[SEDI_PWM_ID_NUM];
};

struct block_runtime block_rt[SEDI_PWM_NUM] = {
	{ .instance = COUNTER_BLOCK_0, },
	{ .instance = COUNTER_BLOCK_1, },
};

static void counter_block_callback(void *data, uint32_t status)
{
	const struct device *dev = data;
	uint32_t instance = GET_BLOCK_INSTANCE(dev);
	struct block_runtime *rt = dev->data;
	uint32_t counter = 0;

	while (counter < SEDI_PWM_ID_NUM) {
		if ((status & BIT(counter)) && rt->cb[counter]) {
			rt->cb[counter](rt->counter_dev[counter],
					rt->cb_data[counter]);

			/* check if this is a one-shot timer */
			if ((~rt->repeat & BIT(counter))) {
				sedi_pwm_stop(instance, counter);
			}
		}
		counter++;
	}
}

static int counter_pwm_sedi_start(const struct device *dev)
{
	uint32_t counter = GET_COUNTER_INSTANCE(dev);
	uint32_t block = GET_BLOCK_INSTANCE(dev);

	k_mutex_lock(GET_COUNTER_MUTEX(dev), K_FOREVER);
	sedi_pwm_start(block, counter);
	k_mutex_unlock(GET_COUNTER_MUTEX(dev));

	return 0;
}

static int counter_pwm_sedi_stop(const struct device *dev)
{
	uint32_t counter = GET_COUNTER_INSTANCE(dev);
	uint32_t block = GET_BLOCK_INSTANCE(dev);

	k_mutex_lock(GET_COUNTER_MUTEX(dev), K_FOREVER);
	sedi_pwm_stop(block, counter);
	k_mutex_unlock(GET_COUNTER_MUTEX(dev));

	return 0;
}

static int counter_pwm_sedi_get_value(const struct device *dev,
				      uint32_t *counter_value)
{
	uint32_t counter = GET_COUNTER_INSTANCE(dev);
	uint32_t block = GET_BLOCK_INSTANCE(dev);

	k_mutex_lock(GET_COUNTER_MUTEX(dev), K_FOREVER);
	sedi_pwm_get_count(block, counter, counter_value);
	k_mutex_unlock(GET_COUNTER_MUTEX(dev));

	return 0;
}

static int _counter_pwm_sedi_set_top_value(const struct device *dev,
					   uint32_t count,
					   counter_top_callback_t callback,
					   uint32_t periodic, void *user_data)
{

	sedi_pwm_config_t cfg;
	uint32_t counter = GET_COUNTER_INSTANCE(dev);
	uint32_t block = GET_BLOCK_INSTANCE(dev);
	struct block_runtime *rt = dev->data;

	rt->cb[counter] = callback;
	rt->cb_data[counter] = user_data;

	if (periodic) {
		rt->repeat |= BIT(counter);
	} else {
		rt->repeat &= ~BIT(counter);
	}

	cfg.mode = SEDI_PWM_MODE_TIMER_COUNT;
	cfg.hi_count = 0;
	cfg.lo_count = count;
	cfg.intr_enable = true;

	k_mutex_lock(GET_COUNTER_MUTEX(dev), K_FOREVER);
	sedi_pwm_stop(block, counter);
	sedi_pwm_set_config(block, counter, cfg);
	sedi_pwm_start(block, counter);
	k_mutex_unlock(GET_COUNTER_MUTEX(dev));
	return 0;
}

static inline int counter_pwm_sedi_set_top_value(const struct device *dev,
						 const struct counter_top_cfg *cfg)
{
	return _counter_pwm_sedi_set_top_value(dev, cfg->ticks, cfg->callback,
					       cfg->flags, cfg->user_data);
}

static uint32_t counter_pwm_sedi_get_pending_int(const struct device *dev)
{
	ARG_UNUSED(dev);
	return -ENOTSUP; /* Not supported */
}

static int counter_pwm_sedi_set_alarm(const struct device *dev, uint8_t chan_id,
				      const struct counter_alarm_cfg *alarm_cfg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan_id);
	ARG_UNUSED(alarm_cfg);
	return -ENOTSUP; /* Not supported */
}

static int counter_pwm_sedi_cancel_alarm(const struct device *dev,
					 uint8_t chan_id)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan_id);
	return -ENOTSUP;  /* Not supported */
}

static const struct counter_driver_api counter_pwm_sedi_api = {
	.start = counter_pwm_sedi_start,
	.stop = counter_pwm_sedi_stop,
	.get_value = counter_pwm_sedi_get_value,
	.set_top_value = counter_pwm_sedi_set_top_value,
	.get_pending_int = counter_pwm_sedi_get_pending_int,
	.set_alarm = counter_pwm_sedi_set_alarm,
	.cancel_alarm = counter_pwm_sedi_cancel_alarm
};

static int counter_init(const struct device *dev)
{
	struct block_runtime *rt = dev->data;
	struct sedi_counter_cfg *config =
		(struct sedi_counter_cfg *) dev->config;
	uint32_t counter = GET_COUNTER_INSTANCE(dev);

	k_mutex_init(GET_COUNTER_MUTEX(dev));
	config->info.freq = sedi_pm_get_lbw_clock();

	counter = counter % SEDI_PWM_ID_NUM;
	rt->counter_dev[counter] = dev;
	rt->cb[counter] = NULL;
	rt->cb_data[counter] = NULL;

	return 0;
}

static int counter_block_init(const struct device *dev)
{
	uint32_t instance = GET_BLOCK_INSTANCE(dev);
	int ret;
	struct block_config *config = (struct block_config *)dev->config;

	ret = sedi_pwm_init(instance, counter_block_callback, (void *)dev);
	if (ret != SEDI_DRIVER_OK) {
		return -ENODEV;
	}

	config->irq_config_func(dev);

	return 0;
}

#if DT_NODE_HAS_STATUS(DT_NODELABEL(pwm0), okay)
COUNTER_DEVICE_INIT(0, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
COUNTER_DEVICE_INIT(1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
COUNTER_DEVICE_INIT(2, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
COUNTER_DEVICE_INIT(3, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
COUNTER_DEVICE_INIT(4, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
COUNTER_DEVICE_INIT(5, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
COUNTER_DEVICE_INIT(6, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
COUNTER_DEVICE_INIT(7, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(pwm1), okay)
COUNTER_DEVICE_INIT(8, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
COUNTER_DEVICE_INIT(9, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
COUNTER_DEVICE_INIT(10, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
COUNTER_DEVICE_INIT(11, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
COUNTER_DEVICE_INIT(12, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
COUNTER_DEVICE_INIT(13, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
COUNTER_DEVICE_INIT(14, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
COUNTER_DEVICE_INIT(15, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
#endif

DT_INST_FOREACH_STATUS_OKAY(COUNTER_SEDI_BLOCK_INIT)
