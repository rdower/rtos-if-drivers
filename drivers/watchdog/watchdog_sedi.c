/*
 * Copyright (c) 2018-2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <drivers/watchdog.h>
#include <kernel.h>
#include <sedi.h>

#define DT_DRV_COMPAT intel_pse_watchdog

struct watchdog_context {
	int sedi_device;
	struct k_mutex *mutex;
	int err;
	struct wdt_timeout_cfg cfg;
};

/*
 * According to API description, the config should not set into
 * sedi before setup called.
 */
static int watchdog_api_install_timeout(
	const struct device *dev,
	const struct wdt_timeout_cfg *cfg
	)
{
	__ASSERT(dev != NULL, "");
	__ASSERT(cfg != NULL, "");

	int ret;
	struct watchdog_context *context = (void *)dev->data;
	int max_ms = MS_PER_SEC * sedi_watchdog_get_max_timeout(
		context->sedi_device);

	if (cfg->flags != WDT_FLAG_RESET_SOC) {
		return -ENOTSUP;
	}

	ret = k_mutex_lock(context->mutex, K_FOREVER);
	if (ret != 0) {
		return -EIO;
	}

	if ((cfg->window.min == 0) || (cfg->window.max == 0) ||
	    (cfg->window.min > max_ms) || (cfg->window.max > max_ms)) {
		return -EINVAL;
	}

	context->cfg.window.min = cfg->window.min;
	context->cfg.window.max = cfg->window.max;
	context->cfg.callback = cfg->callback;

	k_mutex_unlock(context->mutex);

	return 0;
}

static void watchdog_sedi_callback(uint32_t event, const void *arg)
{
	ARG_UNUSED(event);

	struct device *dev = (struct device *)arg;
	struct watchdog_context *context = (void *)dev->data;

	/* channel is not supported */
	if (context->cfg.callback) {
		context->cfg.callback(dev, 0);
	}
}

static int watchdog_api_setup(const struct device *dev, uint8_t options)
{
	__ASSERT(dev != NULL, "");

	int ret;
	struct watchdog_context *context = (void *)dev->data;
	sedi_watchdog_config_t sedi_config;

	ret = k_mutex_lock(context->mutex, K_FOREVER);
	if (ret != 0) {
		goto failed_before_lock;
	}

	ret = sedi_watchdog_init(
		context->sedi_device,
		watchdog_sedi_callback,
		dev
		);
	if (ret != 0) {
		goto failed;
	}

	sedi_config.ms_low = context->cfg.window.min;
	sedi_config.ms_high = context->cfg.window.max;

	ret = sedi_watchdog_config(
		context->sedi_device,
		&sedi_config
		);
	if (ret != 0) {
		goto failed;
	}

	sedi_watchdog_pause_in_sleep(options & WDT_OPT_PAUSE_IN_SLEEP);
	ret = sedi_watchdog_enable(context->sedi_device);
	if (ret != 0) {
		goto failed;
	}

	k_mutex_unlock(context->mutex);

	return 0;

failed:
	k_mutex_unlock(context->mutex);

failed_before_lock:
	return -EIO;
}

static int watchdog_api_feed(const struct device *dev, int channel_id)
{
	__ASSERT(dev != NULL, "");

	(void)channel_id;

	int ret;
	struct watchdog_context *context = (void *)dev->data;

	ret = sedi_watchdog_feed(context->sedi_device);
	if (ret != 0) {
		return -EIO;
	}

	return 0;
}

static int watchdog_api_disable(const struct device *dev)
{
	__ASSERT(dev != NULL, "");

	int ret;
	struct watchdog_context *context = (void *)dev->data;

	ret = k_mutex_lock(context->mutex, K_FOREVER);
	if (ret != 0) {
		goto failed_before_lock;
	}

	ret = sedi_watchdog_disable(context->sedi_device);
	if (ret != 0) {
		goto failed;
	}

	k_mutex_unlock(context->mutex);

	return 0;

failed:
	k_mutex_unlock(context->mutex);

failed_before_lock:
	return -EIO;
}

static const struct wdt_driver_api watchdog_apis = {
	.setup = watchdog_api_setup,
	.disable = watchdog_api_disable,
	.install_timeout = watchdog_api_install_timeout,
	.feed = watchdog_api_feed
};

/* the symbol comes from sedi driver */
extern void sedi_watchdog_0_isr(IN sedi_watchdog_t watchdog_device);

#define CREATE_WATCHDOG_INSTANCE(num)				    \
	static K_MUTEX_DEFINE(watchdog_##num##_mutex);		    \
	static struct watchdog_context watchdog_##num##_context = { \
		.sedi_device = num,				    \
		.mutex = &watchdog_##num##_mutex		    \
	};							    \
	static int watchdog_##num##_init(const struct device *dev)  \
	{							    \
		IRQ_CONNECT(DT_INST_IRQN(num),			    \
			    DT_INST_IRQ(num, priority),		    \
			    sedi_watchdog_##num##_isr,		    \
			    num,				    \
			    0);					    \
		/* assume it always successful as CONNECT ok */	    \
		irq_enable(DT_INST_IRQN(num));			    \
		return 0;					    \
	}							    \
	DEVICE_DEFINE(					    \
		watchdog_sedi_##num,				    \
		"WATCHDOG_" # num,				    \
		watchdog_##num##_init,				    \
		NULL,						    \
		&watchdog_##num##_context,			    \
		NULL,						    \
		POST_KERNEL,					    \
		CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		    \
		&watchdog_apis					    \
		)

CREATE_WATCHDOG_INSTANCE(0);
