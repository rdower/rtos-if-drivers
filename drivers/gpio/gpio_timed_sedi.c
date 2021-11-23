/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <device.h>
#include <stdio.h>
#include <drivers/gpio-timed.h>
#include <pm/pm.h>
#include <syscall_handler.h>
#include "driver/sedi_driver_tgpio.h"

#define DT_DRV_COMPAT intel_pse_tgpio

#define TGPIO_GET_INSTANCE(dev)			      \
	((sedi_tgpio_t)((const struct tgpio_config *) \
			dev->config)->instance)

#define SEM_GET(dev) (&((struct tgpio_runtime *)(dev->data))->sem)

struct tgpio_config {
	int instance;
};

struct tgpio_runtime {
	struct k_sem sem;
	tgpio_pin_callback_t cb_list[SEDI_GPIO_PIN_NUM];
	uint32_t pin_busy;
	void (*irq_config_func)(const struct device *dev);
#ifdef CONFIG_PM_DEVICE
	uint32_t device_power_state;
#endif
};

void callback(const void *data, uint32_t status)
{
	unsigned int pin = 0;
	uint64_t ec;
	sedi_tgpio_time_t ts;
	struct tgpio_time tstamp;
	const struct device *port = (const struct device *)data;

	__ASSERT(port != NULL, "");
	sedi_tgpio_t instance = TGPIO_GET_INSTANCE(port);
	struct tgpio_runtime *rt = port->data;

	__ASSERT(rt != NULL, "");

	while (pin < SEDI_GPIO_PIN_NUM) {
		if ((status & BIT(pin)) && rt->cb_list[pin]) {
			sedi_tgpio_get_pindata(instance, pin, &ts, &ec);
			tstamp.sec = ts.sec;
			tstamp.nsec = ts.nsec;
			rt->cb_list[pin](port, pin, tstamp, ec);
		}
		pin++;
	}
}

#ifdef CONFIG_PM_DEVICE
static int tgpio_suspend_device(const struct device *port)
{
	uint32_t status;
	sedi_tgpio_t instance = TGPIO_GET_INSTANCE(port);

	if (pm_device_is_busy(port)) {
		return -EBUSY;
	}

	status = sedi_tgpio_set_power(instance, SEDI_POWER_SUSPEND);
	if (status != SEDI_DRIVER_OK) {
		return -EIO;
	}

	return 0;
}

static int tgpio_resume_device_from_suspend(const struct device *port)
{
	uint32_t status;
	sedi_tgpio_t instance = TGPIO_GET_INSTANCE(port);

	status = sedi_tgpio_set_power(instance, SEDI_POWER_FULL);
	if (status != SEDI_DRIVER_OK) {
		return -EIO;
	}

	return 0;
}

static int tgpio_set_device_low_power(const struct device *port)
{
	uint32_t status;
	sedi_tgpio_t instance = TGPIO_GET_INSTANCE(port);

	if (pm_device_is_busy(port)) {
		return -EBUSY;
	}

	status = sedi_tgpio_set_power(instance, SEDI_POWER_LOW);
	if (status != SEDI_DRIVER_OK) {
		return -EIO;
	}

	return 0;
}

static int tgpio_force_suspend_device(const struct device *port)
{
	sedi_tgpio_t instance = TGPIO_GET_INSTANCE(port);

	sedi_tgpio_set_power(instance, SEDI_POWER_FORCE_SUSPEND);

	return 0;
}

static int tgpio_device_action_cb(const struct device *port,
				  enum pm_device_action action)
{
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		ret = tgpio_suspend_device(port);
		break;
	case PM_DEVICE_ACTION_LOW_POWER:
		ret = tgpio_set_device_low_power(port);
		break;
	case PM_DEVICE_ACTION_RESUME:
		ret = tgpio_resume_device_from_suspend(port);
		break;
	case PM_DEVICE_ACTION_FORCE_SUSPEND:
		tgpio_force_suspend_device(port);
		break;
	default:
		ret = -ENOTSUP;
	}

	return ret;
}
#endif

static inline int tgpio_sedi_set_time(const struct device *port,
				      enum tgpio_timer timer, uint32_t sec,
				      uint32_t ns)
{
	int status;
	k_timeout_t flag;
	sedi_tgpio_t instance = TGPIO_GET_INSTANCE(port);

	flag = k_is_in_isr() ? K_NO_WAIT : K_FOREVER;
	if (k_sem_take(SEM_GET(port), flag)) {
		return -EBUSY;
	}
	status = sedi_tgpio_set_time(instance, timer, sec, ns);
	k_sem_give(SEM_GET(port));

	return status;
}

static inline int tgpio_sedi_get_time(const struct device *port,
				      enum tgpio_timer timer, uint32_t *sec,
				      uint32_t *ns)
{
	int status;
	k_timeout_t flag;
	sedi_tgpio_t instance = TGPIO_GET_INSTANCE(port);

	flag = k_is_in_isr() ? K_NO_WAIT : K_FOREVER;
	if (k_sem_take(SEM_GET(port), flag)) {
		return -EBUSY;
	}
	status = sedi_tgpio_get_time(instance, timer, (uint32_t *)sec,
				     (uint32_t *)ns);
	k_sem_give(SEM_GET(port));

	return status;
}

static inline int tgpio_sedi_adjust_time(const struct device *port,
					 enum tgpio_timer timer, int32_t nsec)
{
	int status;
	k_timeout_t flag;
	sedi_tgpio_t instance = TGPIO_GET_INSTANCE(port);

	flag = k_is_in_isr() ? K_NO_WAIT : K_FOREVER;
	if (k_sem_take(SEM_GET(port), flag)) {
		return -EBUSY;
	}

	status = sedi_tgpio_adjust_time(instance, timer, nsec);
	k_sem_give(SEM_GET(port));

	return status;
}

static inline int tgpio_sedi_adjust_frequency(const struct device *port,
					      enum tgpio_timer timer,
					      int32_t ppb)
{
	int status;
	k_timeout_t flag;
	sedi_tgpio_t instance = TGPIO_GET_INSTANCE(port);

	flag = k_is_in_isr() ? K_NO_WAIT : K_FOREVER;
	if (k_sem_take(SEM_GET(port), flag)) {
		return -EBUSY;
	}
	status = sedi_tgpio_adjust_frequency(instance, timer, ppb);
	k_sem_give(SEM_GET(port));

	return status;
}

static inline int tgpio_sedi_get_cross_timestamp(const struct device *port,
						 enum tgpio_timer local_clock,
						 enum tgpio_timer ref_clock,
						 struct tgpio_time *local_time,
						 struct tgpio_time *ref_time)
{
	int status;
	k_timeout_t flag;
	sedi_tgpio_t instance = TGPIO_GET_INSTANCE(port);
	sedi_tgpio_time_t local, reference;

	flag = k_is_in_isr() ? K_NO_WAIT : K_FOREVER;
	if (k_sem_take(SEM_GET(port), flag)) {
		return -EBUSY;
	}
	status = sedi_tgpio_get_cross_timestamp(instance, local_clock,
						ref_clock, &local, &reference);
	if (status) {
		k_sem_give(SEM_GET(port));
		return status;
	}

	local_time->nsec = local.nsec;
	local_time->sec = local.sec;
	ref_time->nsec = reference.nsec;
	ref_time->sec = reference.sec;

	k_sem_give(SEM_GET(port));

	return status;
}

static inline int tgpio_sedi_pin_enable(const struct device *port, uint32_t pin)
{
	sedi_tgpio_t instance = TGPIO_GET_INSTANCE(port);
	struct tgpio_runtime *rt = port->data;

	rt->pin_busy |= BIT(pin);
	pm_device_busy_set(port);
	return sedi_tgpio_pin_enable(instance, pin);
}

static inline int tgpio_sedi_periodic_output(const struct device *port,
					     uint32_t pin,
					     enum tgpio_timer timer,
					     struct tgpio_time *start_time,
					     struct tgpio_time *repeat_interval,
					     struct more_args *margs)
{
	int status;
	k_timeout_t flag;
	struct tgpio_runtime *rt = port->data;
	sedi_tgpio_pin_config_t cfg = { 0 };
	sedi_tgpio_t instance = TGPIO_GET_INSTANCE(port);

	cfg.timer = timer;
	cfg.ev_polarity = TGPIO_RISING_EDGE;
	cfg.intr_enable = 0;
	cfg.direction = SEDI_OUTPUT;
	cfg.start_time.nsec = start_time->nsec;
	cfg.start_time.sec = start_time->sec;
	cfg.repeat_interval.nsec = repeat_interval->nsec;
	cfg.repeat_interval.sec = repeat_interval->sec;
	cfg.repeat_count = margs->arg0;
	cfg.pulse_width = margs->arg1;
	rt->cb_list[pin] = NULL;

	flag = k_is_in_isr() ? K_NO_WAIT : K_FOREVER;
	if (k_sem_take(SEM_GET(port), flag)) {
		return -EBUSY;
	}
	status = sedi_tgpio_pin_config(instance, pin, cfg);
	if (status) {
		k_sem_give(SEM_GET(port));
		return status;
	}

	tgpio_sedi_pin_enable(port, pin);
	k_sem_give(SEM_GET(port));

	return status;
}

static inline int tgpio_sedi_external_timestamp(const struct device *port,
						uint32_t pin,
						enum tgpio_timer timer,
						uint32_t events_ceiling,
						enum tgpio_pin_polarity edge,
						tgpio_pin_callback_t cb)
{
	int status;
	k_timeout_t flag;
	struct tgpio_runtime *rt = port->data;
	sedi_tgpio_pin_config_t cfg = { 0 };
	sedi_tgpio_t instance = TGPIO_GET_INSTANCE(port);

	cfg.timer = timer;
	cfg.ev_polarity = edge;
	cfg.intr_enable = 1;
	cfg.direction = SEDI_INPUT;
	cfg.events_ceiling = events_ceiling;
	rt->cb_list[pin] = cb;

	flag = k_is_in_isr() ? K_NO_WAIT : K_FOREVER;
	if (k_sem_take(SEM_GET(port), flag)) {
		return -EBUSY;
	}
	status = sedi_tgpio_pin_config(instance, pin, cfg);
	if (status) {
		k_sem_give(SEM_GET(port));
		return status;
	}

	tgpio_sedi_pin_enable(port, pin);
	k_sem_give(SEM_GET(port));


	return status;
}

static inline int tgpio_sedi_count_events(const struct device *port,
					  uint32_t pin,
					  enum tgpio_timer timer,
					  struct tgpio_time start_time,
					  struct tgpio_time repeat_interval,
					  enum tgpio_pin_polarity edge,
					  tgpio_pin_callback_t cb)
{
	int status;
	k_timeout_t flag;
	struct tgpio_runtime *rt = port->data;
	sedi_tgpio_pin_config_t cfg = { 0 };
	sedi_tgpio_t instance = TGPIO_GET_INSTANCE(port);

	cfg.timer = timer;
	cfg.ev_polarity = edge;
	cfg.intr_enable = 1;
	cfg.direction = SEDI_INPUT;
	cfg.start_time.nsec = start_time.nsec;
	cfg.start_time.sec = start_time.sec;
	cfg.repeat_interval.nsec = repeat_interval.nsec;
	cfg.repeat_interval.sec = repeat_interval.sec;
	rt->cb_list[pin] = cb;

	flag = k_is_in_isr() ? K_NO_WAIT : K_FOREVER;
	if (k_sem_take(SEM_GET(port), flag)) {
		return -EBUSY;
	}
	status = sedi_tgpio_pin_config(instance, pin, cfg);
	if (status) {
		k_sem_give(SEM_GET(port));
		return status;
	}

	tgpio_sedi_pin_enable(port, pin);
	k_sem_give(SEM_GET(port));


	return status;
}

static inline int tgpio_sedi_pin_disable(const struct device *port,
					 uint32_t pin)
{
	int status;
	sedi_tgpio_t instance = TGPIO_GET_INSTANCE(port);
	struct tgpio_runtime *rt = port->data;

	status =  sedi_tgpio_pin_disable(instance, pin);

	rt->pin_busy &= ~BIT(pin);
	if (!rt->pin_busy) {
		pm_device_busy_clear(port);
	}

	return status;
}

static const struct tgpio_driver_api api_funcs = {
	.pin_disable = tgpio_sedi_pin_disable,
	.pin_enable = tgpio_sedi_pin_enable,
	.set_time = tgpio_sedi_set_time,
	.get_time = tgpio_sedi_get_time,
	.adjust_time = tgpio_sedi_adjust_time,
	.adjust_frequency = tgpio_sedi_adjust_frequency,
	.get_cross_timestamp = tgpio_sedi_get_cross_timestamp,
	.set_perout = tgpio_sedi_periodic_output,
	.ext_ts = tgpio_sedi_external_timestamp,
	.count_events = tgpio_sedi_count_events,
};

static int tgpio_init(const struct device *port)
{
	int status, instance;
	struct tgpio_runtime *rt = port->data;

	k_sem_init(SEM_GET(port), 1, 1);
	instance = TGPIO_GET_INSTANCE(port);

	status = sedi_tgpio_init(instance, callback, (void *)port);
	if (status != SEDI_DRIVER_OK) {
		return -ENODEV;
	}

	rt->irq_config_func(port);

	return status;
}

#define TGPIO_SEDI_DEVICE_INIT(n)					\
	static void irq_config_##n(const struct device *dev)		\
	{								\
		ARG_UNUSED(dev);					\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority),			\
			    sedi_tgpio_isr_handler,			\
			    DT_INST_PROP(n, peripheral_id), 0);		\
		irq_enable(DT_INST_IRQN(n));				\
	}								\
	static struct tgpio_runtime tgpio_##n##_runtime = {		\
		.irq_config_func = irq_config_##n,			\
	};								\
	static const struct tgpio_config tgpio_##n##_cfg = {		\
		.instance = DT_INST_PROP(n, peripheral_id),		\
	};								\
	DEVICE_DEFINE(tgpio_##n, DT_INST_LABEL(n), &tgpio_init,		\
		      tgpio_device_action_cb, &tgpio_##n##_runtime,	\
		      &tgpio_##n##_cfg,					\
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,	\
		      &api_funcs);

DT_INST_FOREACH_STATUS_OKAY(TGPIO_SEDI_DEVICE_INIT)
