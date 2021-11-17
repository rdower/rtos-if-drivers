/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <drivers/sideband.h>
#include <kernel.h>
#include <sedi.h>

#define DT_DRV_COMPAT intel_pse_sideband

struct sideband_sedi_data {
	struct k_mutex *lock;
	struct k_sem *sync;
};

extern void sedi_sb_isr(sedi_sideband_t sideband_device);

#ifdef CONFIG_PM_DEVICE

static uint32_t sideband_resume_device_from_suspend(const struct device *dev)
{
	uint32_t status;

	status = sedi_sideband_set_power(SEDI_SIDEBAND_0, SEDI_POWER_FULL);
	if (status != SEDI_DRIVER_OK) {
		return -EIO;
	}

	return 0;
}

static int sideband_sedi_device_action_cb(const struct device *dev, enum pm_device_action action)
{
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		/* Nothing to do */
		break;
	case PM_DEVICE_ACTION_RESUME:
		ret = sideband_resume_device_from_suspend(dev);
		break;
	case PM_DEVICE_ACTION_LOW_POWER:
		/* Always-on module, nothing to do */
		break;

	case PM_DEVICE_ACTION_FORCE_SUSPEND:
		/* Nothing to do */
		break;
	default:
		return -ENOTSUP;
	}
	return ret;
}

#endif /* CONFIG_PM_DEVICE */


static void callback(uint32_t event, void *param)
{
	struct device *dev = (struct device *)param;
	struct sideband_sedi_data *context = dev->data;

	/* Give semaphore */
	k_sem_give(context->sync);
}

static int sideband_sedi_send(const struct device *dev, uint32_t port,
			      uint32_t action, uint64_t addr, uint32_t data)
{
	int ret;
	struct sideband_sedi_data *context = dev->data;

	k_mutex_lock(context->lock, K_FOREVER);
	ret = sedi_sideband_send(SEDI_SIDEBAND_0, port, action, addr, data);
	k_mutex_unlock(context->lock);

	return ret;
}

static int sideband_sedi_receive(const struct device *dev, uint32_t port,
				 uint64_t *addr, uint64_t *data)
{
	int ret;
	struct sideband_sedi_data *context = dev->data;

	k_mutex_lock(context->lock, K_FOREVER);
	ret = sedi_sideband_recv_async(SEDI_SIDEBAND_0, port, addr, data);

	/* Wait for semaphore give */
	k_sem_take(context->sync, K_FOREVER);
	k_mutex_unlock(context->lock);

	return ret;
}

static int sideband_sedi_write_raw(const struct device *dev,
				   struct sb_raw_message *message)
{
	int ret;
	struct sideband_sedi_data *context = dev->data;

	k_mutex_lock(context->lock, K_FOREVER);
	ret = sedi_sideband_upstream_write_raw(SEDI_SIDEBAND_0,
					       message->addr_low,
					       message->addr_high,
					       message->attr, message->data0,
					       message->data1, message->sairs,
					       SEDI_SIDEBAND_ACTION_WRITE);
	k_mutex_unlock(context->lock);

	return ret;
}

static int sideband_sedi_read_raw(const struct device *dev,
				  struct sb_raw_message *message)
{
	int ret;
	struct sideband_sedi_data *context = dev->data;

	k_mutex_lock(context->lock, K_FOREVER);
	ret = sedi_sideband_downstream_read_raw(SEDI_SIDEBAND_0,
						&message->addr_low,
						&message->addr_high,
						&message->attr,
						&message->data0,
						&message->data1,
						&message->sairs);
	k_mutex_unlock(context->lock);

	return ret;
}

static int sideband_sedi_send_ack(const struct device *dev, uint32_t port,
				  uint32_t action, uint32_t data)
{
	int ret;
	struct sideband_sedi_data *context = dev->data;

	k_mutex_lock(context->lock, K_FOREVER);
	ret = sedi_sideband_send_ack(SEDI_SIDEBAND_0, port, action,
				     SEDI_SIDEBAND_RESPONSE_COMPLETE, data);
	k_mutex_unlock(context->lock);

	return ret;
}


static int sideband_sedi_wait_ack(const struct device *dev, uint32_t port,
				  uint32_t action, uint32_t *data)
{
	int ret;
	struct sideband_sedi_data *context = dev->data;

	k_mutex_lock(context->lock, K_FOREVER);
	ret = sedi_sideband_wait_ack(SEDI_SIDEBAND_0, port, action, data);
	k_mutex_unlock(context->lock);

	return ret;
}

static int sideband_sedi_register_client(const struct device *dev,
					 uint32_t client, uint32_t opcode)
{
	int ret;
	struct sideband_sedi_data *context = dev->data;

	k_mutex_lock(context->lock, K_FOREVER);
	ret = sedi_sideband_register_client(SEDI_SIDEBAND_0, client, opcode,
					    callback, (void *)dev);
	k_mutex_unlock(context->lock);

	return ret;
}

static int sideband_sedi_unregister_client(const struct device *dev,
					   uint32_t client)
{
	int ret;
	struct sideband_sedi_data *context = dev->data;

	k_mutex_lock(context->lock, K_FOREVER);
	ret = sedi_sideband_unregister_client(SEDI_SIDEBAND_0, client);
	k_mutex_unlock(context->lock);

	return ret;
}

static const struct sideband_driver_api sedi_sideband_driver_api = {
	.send = sideband_sedi_send,
	.receive = sideband_sedi_receive,
	.write_raw = sideband_sedi_write_raw,
	.read_raw = sideband_sedi_read_raw,
	.send_ack = sideband_sedi_send_ack,
	.wait_ack = sideband_sedi_wait_ack,
	.register_client = sideband_sedi_register_client,
	.unregister_client = sideband_sedi_unregister_client
};

static struct sideband_sedi_data sideband_data;
static K_SEM_DEFINE(sideband_sem, 0, 1);
static K_MUTEX_DEFINE(sideband_lock);

static int sideband_sedi_init(const struct device *dev)
{
	struct sideband_sedi_data *context = dev->data;

	sedi_sideband_init(SEDI_SIDEBAND_0);
	sedi_sideband_set_power(SEDI_SIDEBAND_0, SEDI_POWER_FULL);

	/* Connect irq */
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority),
		    sedi_sb_isr, 0, 0);
	irq_enable(DT_INST_IRQN(0));

	context->lock = &sideband_lock;
	context->sync = &sideband_sem;
	return 0;
}

DEVICE_DEFINE(sideband, "SIDEBAND", &sideband_sedi_init,
	      sideband_sedi_device_action_cb, &sideband_data, NULL, POST_KERNEL,
	      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &sedi_sideband_driver_api);
