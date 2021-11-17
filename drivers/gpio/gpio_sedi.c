/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sedi_driver_gpio.h"
#include <drivers/gpio.h>
#include <kernel.h>
#include <soc.h>
#include "gpio_utils.h"

#define DT_DRV_COMPAT intel_pse_gpio

#define PINS_PER_PORT (32)

struct gpio_sedi_config {
	struct gpio_driver_config common;
	sedi_gpio_t device;
	sedi_gpio_port_t port;
	uint32_t pin_nums;
};

struct gpio_sedi_runtime {
	struct gpio_driver_config common;
	sys_slist_t callbacks;
#ifdef CONFIG_PM_DEVICE
	uint32_t device_power_state;
#endif  /* CONFIG_PM_DEVICE */
};

static int gpio_sedi_init(const struct device *dev);

#ifdef CONFIG_PM_DEVICE

static int gpio_suspend_device(const struct device *dev)
{
	uint32_t status;
	const struct gpio_sedi_config *config = dev->config;

	if (pm_device_is_busy(dev)) {
		return -EBUSY;
	}

	status = sedi_gpio_set_power(config->device, SEDI_POWER_SUSPEND);
	if (status != SEDI_DRIVER_OK) {
		return -EIO;
	}

	return 0;
}

static int gpio_resume_device_from_suspend(const struct device *dev)
{
	uint32_t status;
	const struct gpio_sedi_config *config = dev->config;

	status = sedi_gpio_set_power(config->device, SEDI_POWER_FULL);
	if (status != SEDI_DRIVER_OK) {
		return -EIO;
	}

	return 0;
}

static int gpio_set_device_low_power(const struct device *dev)
{
	uint32_t status;
	const struct gpio_sedi_config *config = dev->config;

	if (pm_device_is_busy(dev)) {
		return -EBUSY;
	}

	status = sedi_gpio_set_power(config->device, SEDI_POWER_LOW);
	if (status != SEDI_DRIVER_OK) {
		return -EIO;
	}

	return 0;
}

static int gpio_force_suspend_device(const struct device *dev)
{
	uint32_t status;
	const struct gpio_sedi_config *config = dev->config;

	status = sedi_gpio_set_power(config->device, SEDI_POWER_FORCE_SUSPEND);
	if (status != SEDI_DRIVER_OK) {
		return -EIO;
	}

	return 0;
}

static int gpio_sedi_power_management(const struct device *dev, enum pm_device_action action)
{
	int ret = 0;

		switch (action) {
		case PM_DEVICE_ACTION_SUSPEND:
			ret = gpio_suspend_device(dev);
			break;
		case PM_DEVICE_ACTION_RESUME:
			ret = gpio_resume_device_from_suspend(dev);
			break;
		case PM_DEVICE_ACTION_FORCE_SUSPEND:
			gpio_force_suspend_device(dev);
			break;
		case PM_DEVICE_ACTION_LOW_POWER:
			ret = gpio_set_device_low_power(dev);
			break;

		default:
			ret = -ENOTSUP;
		}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

static void gpio_sedi_write_raw(const struct device *dev, uint32_t pins, bool is_clear)
{
	uint8_t i;
	const struct gpio_sedi_config *config = dev->config;
	sedi_gpio_t gpio_dev = config->device;
	sedi_gpio_pin_state_t val;
	uint32_t real_pin;

	if (is_clear) {
		val = GPIO_STATE_LOW;
	} else {
		val = GPIO_STATE_HIGH;
	}

	for (i = 0; i < config->pin_nums; i++) {
		if (pins & 0x1) {
			real_pin = config->port * PINS_PER_PORT + i;
			sedi_gpio_write_pin(gpio_dev, real_pin, val);
		}
		pins >>= 1;
		if (pins == 0) {
			break;
		}
	}
}

static int gpio_sedi_configure(const struct device *dev, gpio_pin_t pin,
			       gpio_flags_t flags)
{
	const struct gpio_sedi_config *config = dev->config;
	sedi_gpio_t gpio_dev = config->device;
	sedi_gpio_pin_config_t pin_config = { 0 };
	uint32_t real_pin = PINS_PER_PORT * config->port + pin;

	/* Output GPIO cannot produce interrupt */
	if ((flags & GPIO_INPUT) && (flags & GPIO_OUTPUT)) {
		return -EINVAL;
	}

	pin_config.enable_interrupt = false;
	/* Map direction */
	if (flags & GPIO_OUTPUT) {
		pin_config.direction = GPIO_DIR_MODE_OUTPUT;
		sedi_gpio_config_pin(gpio_dev, real_pin, pin_config);
		/* Set start state */
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			sedi_gpio_write_pin(gpio_dev, real_pin, 1);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			sedi_gpio_write_pin(gpio_dev, real_pin, 0);
		}
	} else {
		pin_config.direction = GPIO_DIR_MODE_INPUT;
		sedi_gpio_config_pin(gpio_dev, real_pin, pin_config);
	}

	return 0;
}

static int gpio_sedi_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_sedi_config *config = dev->config;
	sedi_gpio_t gpio_dev = config->device;

	*value = sedi_gpio_read_pin_32bits(gpio_dev, 0);

	return 0;
}

static int gpio_sedi_set_masked_raw(const struct device *dev,
				    uint32_t mask,
				    uint32_t value)
{
	gpio_sedi_write_raw(dev, (mask & value), false);

	return 0;
}

static int gpio_sedi_set_bits_raw(const struct device *dev, uint32_t pins)
{
	gpio_sedi_write_raw(dev, pins, false);

	return 0;
}

static int gpio_sedi_clear_bits_raw(const struct device *dev, uint32_t pins)
{
	gpio_sedi_write_raw(dev, pins, true);

	return 0;
}

static int gpio_sedi_toggle_bits(const struct device *dev, uint32_t pins)
{
	const struct gpio_sedi_config *config = dev->config;
	sedi_gpio_t gpio_dev = config->device;
	uint8_t i;
	uint32_t real_pin;

	for (i = 0; i < config->pin_nums; i++) {
		if (pins & 0x1) {
			real_pin = config->port * PINS_PER_PORT + i;
			sedi_gpio_toggle_pin(gpio_dev, real_pin);
		}
		pins >>= 1;
		if (pins == 0) {
			break;
		}
	}

	return 0;
}

static int gpio_sedi_interrupt_configure(const struct device *dev,
					 gpio_pin_t pin, enum gpio_int_mode mode,
					 enum gpio_int_trig trig)
{
	const struct gpio_sedi_config *config = dev->config;
	sedi_gpio_t gpio_dev = config->device;
	sedi_gpio_pin_config_t pin_config = { 0 };
	uint32_t real_pin = PINS_PER_PORT * config->port + pin;

	/* Only input needs interrupt enabled */
	pin_config.direction = GPIO_DIR_MODE_INPUT;
	pin_config.enable_wakeup = true;
	if (mode == GPIO_INT_MODE_DISABLED) {
		pin_config.enable_interrupt = false;
	} else {
		pin_config.enable_interrupt = true;
		if (mode == GPIO_INT_MODE_LEVEL) {
			return -EINVAL;
		}
		switch (trig) {
		case GPIO_INT_TRIG_LOW:
			pin_config.interrupt_mode = GPIO_INT_MODE_FALLING_EDGE;
			break;
		case GPIO_INT_TRIG_HIGH:
			pin_config.interrupt_mode = GPIO_INT_MODE_RISING_EDGE;
			break;
		case GPIO_INT_TRIG_BOTH:
			pin_config.interrupt_mode = GPIO_INT_MODE_BOTH_EDGE;
			break;
		}
	}
	/* Configure interrupt mode */
	sedi_gpio_config_pin(gpio_dev, real_pin, pin_config);

	return 0;
}


static int gpio_sedi_manage_callback(const struct device *dev,
				     struct gpio_callback *callback,
				     bool set)
{
	struct gpio_sedi_runtime *data = dev->data;

	gpio_manage_callback(&(data->callbacks), callback, set);

	return 0;
}

static uint32_t gpio_sedi_get_pending(const struct device *dev)
{
	const struct gpio_sedi_config *config = dev->config;
	sedi_gpio_t gpio_dev = config->device;

	return sedi_gpio_get_gisr(gpio_dev, config->port);
}

static const struct gpio_driver_api gpio_sedi_driver_api = {
	.pin_configure = gpio_sedi_configure,
	.port_get_raw = gpio_sedi_get_raw,
	.port_set_masked_raw = gpio_sedi_set_masked_raw,
	.port_set_bits_raw = gpio_sedi_set_bits_raw,
	.port_clear_bits_raw = gpio_sedi_clear_bits_raw,
	.port_toggle_bits = gpio_sedi_toggle_bits,
	.pin_interrupt_configure = gpio_sedi_interrupt_configure,
	.manage_callback = gpio_sedi_manage_callback,
	.get_pending_int = gpio_sedi_get_pending
};

static const struct gpio_sedi_config gpio0_config = {
	.common = { 0xFFFFFFFF },
	.device = SEDI_GPIO_0,
	.port = GPIO_PORT_0,
	.pin_nums = 30,
};

static struct gpio_sedi_runtime gpio0_runtime;
extern void sedi_gpio_0_isr(void);

DEVICE_DEFINE(gpio_0, DT_PROP(DT_NODELABEL(gpio0), label), &gpio_sedi_init,
	      gpio_sedi_power_management, &gpio0_runtime, &gpio0_config,
	      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
	      &gpio_sedi_driver_api);

static const struct gpio_sedi_config gpio1_config = {
	.common = { 0xFFFFFFFF },
	.device = SEDI_GPIO_1,
	.port = GPIO_PORT_0,
	.pin_nums = 30,
};

static struct gpio_sedi_runtime gpio1_runtime;
extern void sedi_gpio_1_isr(void);


DEVICE_DEFINE(gpio_1, DT_PROP(DT_NODELABEL(gpio1), label), &gpio_sedi_init,
	      gpio_sedi_power_management, &gpio1_runtime, &gpio1_config,
	      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
	      &gpio_sedi_driver_api);

/* Device specific for WOL use case */
static const struct gpio_sedi_config gpio_wol_config = {
	.common = { 0x000000FF },
	.device = SEDI_GPIO_0,
	.port = GPIO_PORT_1,
	.pin_nums = 8,
};

static struct gpio_sedi_runtime gpio_wol_runtime;
DEVICE_DEFINE(gpio_wol, "GPIO_WOL", &gpio_sedi_init,
	      gpio_sedi_power_management, &gpio_wol_runtime, &gpio_wol_config,
	      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
	      &gpio_sedi_driver_api);

static void gpio_sedi_callback(const uint32_t pin_mask, const uint8_t port,
			       void *param)
{
	ARG_UNUSED(port);
	struct device *dev = (struct device *)param;
	struct gpio_sedi_runtime *runtime =
		(struct gpio_sedi_runtime *)(dev->data);

	if (port == 1) {
		runtime = &gpio_wol_runtime;
	}

	/* call the callbacks */
	gpio_fire_callbacks(&runtime->callbacks, dev, pin_mask);

}

static int gpio_sedi_init(const struct device *dev)
{
	int ret = 0;
	const struct gpio_sedi_config *config = dev->config;
	sedi_gpio_t gpio_dev = config->device;

	/* Call sedi gpio init */
	if (config->port == 1) {
		return 0;
	}

	ret = sedi_gpio_init(gpio_dev, gpio_sedi_callback, (void *)dev);

	if (ret != 0) {
		return ret;
	}
	sedi_gpio_set_power(gpio_dev, SEDI_POWER_FULL);
	switch (gpio_dev) {
	case SEDI_GPIO_0:
		IRQ_CONNECT(DT_IRQN(DT_NODELABEL(gpio0)),
			    DT_IRQ(DT_NODELABEL(gpio0), priority),
			    sedi_gpio_0_isr, NULL, 0);
		irq_enable(DT_IRQN(DT_NODELABEL(gpio0)));
		break;
	case SEDI_GPIO_1:
		IRQ_CONNECT(DT_IRQN(DT_NODELABEL(gpio1)),
			    DT_IRQ(DT_NODELABEL(gpio1), priority),
			    sedi_gpio_1_isr, NULL, 0);
		irq_enable(DT_IRQN(DT_NODELABEL(gpio1)));
		break;
	default:
		return -EIO;
	}

	return 0;
}
