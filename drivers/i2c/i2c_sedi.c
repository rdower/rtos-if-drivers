/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <drivers/i2c.h>
#include <kernel.h>
#include <sedi.h>

#define DT_DRV_COMPAT intel_pse_i2c

#define I2C_NOT_USE_DMA (-1)

struct i2c_context {
	int sedi_device;
	struct k_sem *sem;
	struct k_mutex *mutex;
	int err;
	int addr_10bit;
	int tx_dma_instance;
	int rx_dma_instance;
	int tx_channel;
	int rx_cmd_channel;
	int rx_data_channel;
#ifdef CONFIG_PM_DEVICE
	uint32_t device_power_state;
#endif
};

static int i2c_api_configure(const struct device *dev, uint32_t dev_config)
{
	int ret;
	int speed;
	int sedi_speed;
	struct i2c_context *context;

	__ASSERT(dev != NULL, "");

	errno = -EIO;

	speed = I2C_SPEED_GET(dev_config);
	context = (struct i2c_context *)dev->data;

	context->addr_10bit = (dev_config & I2C_ADDR_10_BITS) ? 0x400 : 0;

	if (speed == I2C_SPEED_STANDARD) {
		sedi_speed = SEDI_I2C_BUS_SPEED_STANDARD;
	} else if (speed == I2C_SPEED_FAST) {
		sedi_speed = SEDI_I2C_BUS_SPEED_FAST;
	} else if (speed == I2C_SPEED_FAST_PLUS) {
		sedi_speed = SEDI_I2C_BUS_SPEED_FAST_PLUS;
	} else if (speed == I2C_SPEED_HIGH) {
		sedi_speed = SEDI_I2C_BUS_SPEED_HIGH;
	} else if (speed == I2C_SPEED_ULTRA) {
		/* SEDI interface doesn't have ULTRA speed option */
		sedi_speed = SEDI_I2C_BUS_SPEED_HIGH;
	} else {
		goto error_before_lock;
	}

	ret = k_mutex_lock(context->mutex, K_FOREVER);

	if (ret != 0) {
		goto error_before_lock;
	}

	ret = sedi_i2c_control(
		context->sedi_device,
		SEDI_I2C_BUS_SPEED,
		sedi_speed
		);
	if (ret != 0) {
		goto end;
	}

	errno = 0;

end:
	k_mutex_unlock(context->mutex);

error_before_lock:
	if (errno != 0) {
		return -EIO;
	} else {
		return 0;
	}
}

/* attention: async read or write will wait on semaphore */
static int i2c_api_full_io(
	const struct device *dev,
	struct i2c_msg *msgs,
	uint8_t num_msgs,
	uint16_t addr
	)
{
	int ret = -1;
	struct i2c_context *context = NULL;
	int pending = 1;

	__ASSERT(dev != NULL, "");
	__ASSERT(msgs != NULL, "");

	errno = -EIO;
	context = dev->data;

	ret = k_mutex_lock(context->mutex, K_FOREVER);
	pm_device_busy_set(dev);

	if (ret != 0) {
		goto error_before_lock;
	}

	for (int i = 0; i < num_msgs; i++) {
		if (msgs[i].flags & I2C_MSG_STOP) {
			pending = 0;
		}
		if (0 == (msgs[i].flags & I2C_MSG_RW_MASK)) {
			/* If use DMA transfer */
			if ((context->tx_dma_instance >= 0) &&
			    (context->tx_channel >= 0) &&
			    (msgs[i].len > SEDI_I2C_DMA_LENGTH_LIMIT)) {
				/* User shall notice cache operation */
				ret = sedi_i2c_master_write_dma(
					context->sedi_device,
					context->tx_dma_instance,
					context->tx_channel,
					addr | context->addr_10bit,
					msgs[i].buf, msgs[i].len, pending);
			} else {
				ret = sedi_i2c_master_write_async(
					context->sedi_device,
					addr | context->addr_10bit,
					msgs[i].buf,
					msgs[i].len,
					pending
					);
			}

			if (ret != 0) {
				goto end;
			}
		} else {
			if ((context->rx_dma_instance >= 0) &&
			    (context->rx_data_channel >= 0) &&
			    (context->rx_cmd_channel >= 0) &&
			    (msgs[i].len > SEDI_I2C_DMA_LENGTH_LIMIT)) {
				ret = sedi_i2c_master_read_dma(
					context->sedi_device,
					context->rx_dma_instance,
					context->rx_data_channel,
					context->rx_cmd_channel,
					addr | context->addr_10bit,
					msgs[i].buf, msgs[i].len, pending);
			} else {
				ret = sedi_i2c_master_read_async(
					context->sedi_device,
					addr | context->addr_10bit,
					msgs[i].buf,
					msgs[i].len,
					pending);
			}

			if (ret != 0) {
				goto end;
			}
		}

		ret = k_sem_take(context->sem, K_FOREVER);
		if (ret != 0) {
			goto end;
		}

		if (context->err != 0) {
			goto end;
		}
	}

	errno = 0;
end:
	k_mutex_unlock(context->mutex);

error_before_lock:
	pm_device_busy_clear(dev);
	if (errno != 0) {
		return -EIO;
	} else {
		return 0;
	}
}

static const struct i2c_driver_api i2c_apis = {
	.configure = i2c_api_configure,
	.transfer = i2c_api_full_io
};

extern void dw_i2c_isr(IN sedi_i2c_t i2c_device);

#define CREATE_I2C_INSTANCE(num)				      \
	/* limit 1, as there is no multiple transfer */		      \
	static K_SEM_DEFINE(i2c_##num##_sem, 0, 1);		      \
	static K_MUTEX_DEFINE(i2c_##num##_mutex);		      \
	static struct i2c_context i2c_##num##_context = {	      \
		.sedi_device = num,				      \
		.sem = &i2c_##num##_sem,			      \
		.mutex = &i2c_##num##_mutex,			      \
		.tx_dma_instance = I2C##num##_TX_DMA_DEV,	      \
		.rx_dma_instance = I2C##num##_RX_DMA_DEV,	      \
		.tx_channel = I2C##num##_TX_DMA_CHN,		      \
		.rx_data_channel = I2C##num##_RX_DMA_CHN,	      \
		.rx_cmd_channel = I2C##num##_RX_DMA_CHN_2	      \
	};							      \
	static void i2c_##num##_callback(const uint32_t event)	      \
	{							      \
		if (event == SEDI_I2C_EVENT_TRANSFER_DONE) {	      \
			i2c_##num##_context.err = 0;		      \
		}						      \
		else {						      \
			i2c_##num##_context.err = 1;		      \
		}						      \
		k_sem_give(i2c_##num##_context.sem);		      \
	}							      \
	static int i2c_##num##_init(const struct device *dev)	      \
	{							      \
		int ret;					      \
		ret = sedi_i2c_init(				      \
			i2c_##num##_context.sedi_device,	      \
			i2c_##num##_callback			      \
			);					      \
		if (ret != 0) {					      \
			return -EIO;				      \
		}						      \
		ret = sedi_i2c_set_power(			      \
			i2c_##num##_context.sedi_device,	      \
			SEDI_POWER_FULL);			      \
		if (ret != 0) {					      \
			return -EIO;				      \
		}						      \
		IRQ_CONNECT(DT_IRQN(DT_NODELABEL(i2c##num)),	      \
			    DT_IRQ(DT_NODELABEL(i2c##num), priority), \
			    dw_i2c_isr,				      \
			    num,				      \
			    0);					      \
		/* assume it always successful as CONNECT ok */	      \
		irq_enable(DT_IRQN(DT_NODELABEL(i2c##num)));	      \
		return 0;					      \
	}							      \
	DEVICE_DEFINE(						      \
		i2c_sedi_##num,					      \
		"I2C_" # num,					      \
		i2c_##num##_init,				      \
		i2c_sedi_device_ctrl,				      \
		&i2c_##num##_context,				      \
		NULL,						      \
		POST_KERNEL,					      \
		CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		      \
		&i2c_apis					      \
		)

#ifdef CONFIG_PM_DEVICE

static int i2c_suspend_device(const struct device *dev)
{
	struct i2c_context *context = dev->data;

	if (pm_device_is_busy(dev)) {
		return -EBUSY;
	}

	int ret = sedi_i2c_set_power(context->sedi_device, SEDI_POWER_SUSPEND);

	if (ret != SEDI_DRIVER_OK) {
		return -EIO;
	}


	return 0;
}

static int i2c_resume_device_from_suspend(const struct device *dev)
{
	struct i2c_context *context = dev->data;
	int ret;

	ret = sedi_i2c_set_power(context->sedi_device, SEDI_POWER_FULL);
	if (ret != SEDI_DRIVER_OK) {
		return -EIO;
	}

	pm_device_busy_clear(dev);

	return 0;
}

static int i2c_set_device_low_power(const struct device *dev)
{
	struct i2c_context *context = dev->data;

	if (pm_device_is_busy(dev)) {
		return -EBUSY;
	}

	int ret;

	ret = sedi_i2c_set_power(context->sedi_device, SEDI_POWER_LOW);
	if (ret != SEDI_DRIVER_OK) {
		return -EIO;
	}

	return 0;
}

static int i2c_set_device_force_suspend(const struct device *dev)
{
	struct i2c_context *context = dev->data;
	int ret;

	ret = sedi_i2c_set_power(context->sedi_device,
				 SEDI_POWER_FORCE_SUSPEND);
	if (ret != SEDI_DRIVER_OK) {
		return -EIO;
	}
	return 0;
}

static int i2c_sedi_device_ctrl(const struct device *dev, enum pm_device_action action)
{
	int ret = 0;


		switch (action) {
		case PM_DEVICE_ACTION_SUSPEND:
			ret = i2c_suspend_device(dev);
			break;
		case PM_DEVICE_ACTION_RESUME:
			ret = i2c_resume_device_from_suspend(dev);
			break;
		case PM_DEVICE_ACTION_FORCE_SUSPEND:
			ret = i2c_set_device_force_suspend(dev);
			break;
		case PM_DEVICE_ACTION_LOW_POWER:
			ret = i2c_set_device_low_power(dev);
			break;

		default:
			ret = -ENOTSUP;
		}
	return ret;
}

#endif /* CONFIG_PM_DEVICE */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c0), okay)
#if DT_NODE_HAS_PROP(DT_NODELABEL(i2c0), tx-dma-channel)
#define I2C0_TX_DMA_DEV (DT_PROP(DT_NODELABEL(i2c0), tx-dma-channel) / DMA_CHANNEL_NUM)
#define I2C0_TX_DMA_CHN (DT_PROP(DT_NODELABEL(i2c0), tx-dma-channel) % DMA_CHANNEL_NUM)
#else
#define I2C0_TX_DMA_DEV I2C_NOT_USE_DMA
#define I2C0_TX_DMA_CHN I2C_NOT_USE_DMA
#endif
#if (DT_NODE_HAS_PROP(DT_NODELABEL(i2c0), rx-dma-channel) && \
	DT_NODE_HAS_PROP(DT_NODELABEL(i2c0), rx-dma-ext-channel))
#define I2C0_RX_DMA_DEV (DT_PROP(DT_NODELABEL(i2c0), rx-dma-channel) / DMA_CHANNEL_NUM)
#define I2C0_RX_DMA_CHN (DT_PROP(DT_NODELABEL(i2c0), rx-dma-channel) % DMA_CHANNEL_NUM)
#define I2C0_RX_DMA_DEV_2 (DT_PROP(DT_NODELABEL(i2c0), rx-dma-ext-channel) / DMA_CHANNEL_NUM)
#define I2C0_RX_DMA_CHN_2 (DT_PROP(DT_NODELABEL(i2c0), rx-dma-ext-channel) % DMA_CHANNEL_NUM)
#else
#define I2C0_RX_DMA_DEV I2C_NOT_USE_DMA
#define I2C0_RX_DMA_CHN I2C_NOT_USE_DMA
#define I2C0_RX_DMA_DEV_2 I2C_NOT_USE_DMA
#define I2C0_RX_DMA_CHN_2 I2C_NOT_USE_DMA
#endif
CREATE_I2C_INSTANCE(0);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay)
#if DT_NODE_HAS_PROP(DT_NODELABEL(i2c1), tx-dma-channel)
#define I2C1_TX_DMA_DEV (DT_PROP(DT_NODELABEL(i2c1), tx-dma-channel) / DMA_CHANNEL_NUM)
#define I2C1_TX_DMA_CHN (DT_PROP(DT_NODELABEL(i2c1), tx-dma-channel) % DMA_CHANNEL_NUM)
#else
#define I2C1_TX_DMA_DEV I2C_NOT_USE_DMA
#define I2C1_TX_DMA_CHN I2C_NOT_USE_DMA
#endif
#if (DT_NODE_HAS_PROP(DT_NODELABEL(i2c1), rx-dma-channel) && \
	DT_NODE_HAS_PROP(DT_NODELABEL(i2c1), rx-dma-ext-channel))
#define I2C1_RX_DMA_DEV (DT_PROP(DT_NODELABEL(i2c1), rx-dma-channel) / DMA_CHANNEL_NUM)
#define I2C1_RX_DMA_CHN (DT_PROP(DT_NODELABEL(i2c1), rx-dma-channel) % DMA_CHANNEL_NUM)
#define I2C1_RX_DMA_DEV_2 (DT_PROP(DT_NODELABEL(i2c1), rx-dma-ext-channel) / DMA_CHANNEL_NUM)
#define I2C1_RX_DMA_CHN_2 (DT_PROP(DT_NODELABEL(i2c1), rx-dma-ext-channel) % DMA_CHANNEL_NUM)
#else
#define I2C1_RX_DMA_DEV I2C_NOT_USE_DMA
#define I2C1_RX_DMA_CHN I2C_NOT_USE_DMA
#define I2C1_RX_DMA_DEV_2 I2C_NOT_USE_DMA
#define I2C1_RX_DMA_CHN_2 I2C_NOT_USE_DMA
#endif
CREATE_I2C_INSTANCE(1);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c2), okay)
#if DT_NODE_HAS_PROP(DT_NODELABEL(i2c2), tx-dma-channel)
#define I2C2_TX_DMA_DEV (DT_PROP(DT_NODELABEL(i2c2), tx-dma-channel) / DMA_CHANNEL_NUM)
#define I2C2_TX_DMA_CHN (DT_PROP(DT_NODELABEL(i2c2), tx-dma-channel) % DMA_CHANNEL_NUM)
#else
#define I2C2_TX_DMA_DEV I2C_NOT_USE_DMA
#define I2C2_TX_DMA_CHN I2C_NOT_USE_DMA
#endif
#if (DT_NODE_HAS_PROP(DT_NODELABEL(i2c2), rx-dma-channel) && \
	DT_NODE_HAS_PROP(DT_NODELABEL(i2c2), rx-dma-ext-channel))
#define I2C2_RX_DMA_DEV (DT_PROP(DT_NODELABEL(i2c2), rx-dma-channel) / DMA_CHANNEL_NUM)
#define I2C2_RX_DMA_CHN (DT_PROP(DT_NODELABEL(i2c2), rx-dma-channel) % DMA_CHANNEL_NUM)
#define I2C2_RX_DMA_DEV_2 (DT_PROP(DT_NODELABEL(i2c2), rx-dma-ext-channel) / DMA_CHANNEL_NUM)
#define I2C2_RX_DMA_CHN_2 (DT_PROP(DT_NODELABEL(i2c2), rx-dma-ext-channel) % DMA_CHANNEL_NUM)
#else
#define I2C2_RX_DMA_DEV I2C_NOT_USE_DMA
#define I2C2_RX_DMA_CHN I2C_NOT_USE_DMA
#define I2C2_RX_DMA_DEV_2 I2C_NOT_USE_DMA
#define I2C2_RX_DMA_CHN_2 I2C_NOT_USE_DMA
#endif
CREATE_I2C_INSTANCE(2);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c3), okay)
#if DT_NODE_HAS_PROP(DT_NODELABEL(i2c3), tx-dma-channel)
#define I2C3_TX_DMA_DEV (DT_PROP(DT_NODELABEL(i2c3), tx-dma-channel) / DMA_CHANNEL_NUM)
#define I2C3_TX_DMA_CHN (DT_PROP(DT_NODELABEL(i2c3), tx-dma-channel) % DMA_CHANNEL_NUM)
#else
#define I2C3_TX_DMA_DEV I2C_NOT_USE_DMA
#define I2C3_TX_DMA_CHN I2C_NOT_USE_DMA
#endif
#if (DT_NODE_HAS_PROP(DT_NODELABEL(i2c3), rx-dma-channel) && \
	DT_NODE_HAS_PROP(DT_NODELABEL(i2c3), rx-dma-ext-channel))
#define I2C3_RX_DMA_DEV (DT_PROP(DT_NODELABEL(i2c3), rx-dma-channel) / DMA_CHANNEL_NUM)
#define I2C3_RX_DMA_CHN (DT_PROP(DT_NODELABEL(i2c3), rx-dma-channel) % DMA_CHANNEL_NUM)
#define I2C3_RX_DMA_DEV_2 (DT_PROP(DT_NODELABEL(i2c3), rx-dma-ext-channel) / DMA_CHANNEL_NUM)
#define I2C3_RX_DMA_CHN_2 (DT_PROP(DT_NODELABEL(i2c3), rx-dma-ext-channel) % DMA_CHANNEL_NUM)
#else
#define I2C3_RX_DMA_DEV I2C_NOT_USE_DMA
#define I2C3_RX_DMA_CHN I2C_NOT_USE_DMA
#define I2C3_RX_DMA_DEV_2 I2C_NOT_USE_DMA
#define I2C3_RX_DMA_CHN_2 I2C_NOT_USE_DMA
#endif
CREATE_I2C_INSTANCE(3);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c4), okay)
#if DT_NODE_HAS_PROP(DT_NODELABEL(i2c4), tx-dma-channel)
#define I2C4_TX_DMA_DEV (DT_PROP(DT_NODELABEL(i2c4), tx-dma-channel) / DMA_CHANNEL_NUM)
#define I2C4_TX_DMA_CHN (DT_PROP(DT_NODELABEL(i2c4), tx-dma-channel) % DMA_CHANNEL_NUM)
#else
#define I2C4_TX_DMA_DEV I2C_NOT_USE_DMA
#define I2C4_TX_DMA_CHN I2C_NOT_USE_DMA
#endif
#if (DT_NODE_HAS_PROP(DT_NODELABEL(i2c4), rx-dma-channel) && \
	DT_NODE_HAS_PROP(DT_NODELABEL(i2c4), rx-dma-ext-channel))
#define I2C4_RX_DMA_DEV (DT_PROP(DT_NODELABEL(i2c4), rx-dma-channel) / DMA_CHANNEL_NUM)
#define I2C4_RX_DMA_CHN (DT_PROP(DT_NODELABEL(i2c4), rx-dma-channel) % DMA_CHANNEL_NUM)
#define I2C4_RX_DMA_DEV_2 (DT_PROP(DT_NODELABEL(i2c4), rx-dma-ext-channel) / DMA_CHANNEL_NUM)
#define I2C4_RX_DMA_CHN_2 (DT_PROP(DT_NODELABEL(i2c4), rx-dma-ext-channel) % DMA_CHANNEL_NUM)
#else
#define I2C4_RX_DMA_DEV I2C_NOT_USE_DMA
#define I2C4_RX_DMA_CHN I2C_NOT_USE_DMA
#define I2C4_RX_DMA_DEV_2 I2C_NOT_USE_DMA
#define I2C4_RX_DMA_CHN_2 I2C_NOT_USE_DMA
#endif
CREATE_I2C_INSTANCE(4);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c5), okay)
#if DT_NODE_HAS_PROP(DT_NODELABEL(i2c5), tx-dma-channel)
#define I2C5_TX_DMA_DEV (DT_PROP(DT_NODELABEL(i2c5), tx-dma-channel) / DMA_CHANNEL_NUM)
#define I2C5_TX_DMA_CHN (DT_PROP(DT_NODELABEL(i2c5), tx-dma-channel) % DMA_CHANNEL_NUM)
#else
#define I2C5_TX_DMA_DEV I2C_NOT_USE_DMA
#define I2C5_TX_DMA_CHN I2C_NOT_USE_DMA
#endif
#if (DT_NODE_HAS_PROP(DT_NODELABEL(i2c5), rx-dma-channel) && \
	DT_NODE_HAS_PROP(DT_NODELABEL(i2c5), rx-dma-ext-channel))
#define I2C5_RX_DMA_DEV (DT_PROP(DT_NODELABEL(i2c5), rx-dma-channel) / DMA_CHANNEL_NUM)
#define I2C5_RX_DMA_CHN (DT_PROP(DT_NODELABEL(i2c5), rx-dma-channel) % DMA_CHANNEL_NUM)
#define I2C5_RX_DMA_DEV_2 (DT_PROP(DT_NODELABEL(i2c5), rx-dma-ext-channel) / DMA_CHANNEL_NUM)
#define I2C5_RX_DMA_CHN_2 (DT_PROP(DT_NODELABEL(i2c5), rx-dma-ext-channel) % DMA_CHANNEL_NUM)
#else
#define I2C5_RX_DMA_DEV I2C_NOT_USE_DMA
#define I2C5_RX_DMA_CHN I2C_NOT_USE_DMA
#define I2C5_RX_DMA_DEV_2 I2C_NOT_USE_DMA
#define I2C5_RX_DMA_CHN_2 I2C_NOT_USE_DMA
#endif
CREATE_I2C_INSTANCE(5);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c6), okay)
#if DT_NODE_HAS_PROP(DT_NODELABEL(i2c6), tx-dma-channel)
#define I2C6_TX_DMA_DEV (DT_PROP(DT_NODELABEL(i2c6), tx-dma-channel) / DMA_CHANNEL_NUM)
#define I2C6_TX_DMA_CHN (DT_PROP(DT_NODELABEL(i2c6), tx-dma-channel) % DMA_CHANNEL_NUM)
#else
#define I2C6_TX_DMA_DEV I2C_NOT_USE_DMA
#define I2C6_TX_DMA_CHN I2C_NOT_USE_DMA
#endif
#if (DT_NODE_HAS_PROP(DT_NODELABEL(i2c6), rx-dma-channel) && \
	DT_NODE_HAS_PROP(DT_NODELABEL(i2c6), rx-dma-ext-channel))
#define I2C6_RX_DMA_DEV (DT_PROP(DT_NODELABEL(i2c6), rx-dma-channel) / DMA_CHANNEL_NUM)
#define I2C6_RX_DMA_CHN (DT_PROP(DT_NODELABEL(i2c6), rx-dma-channel) % DMA_CHANNEL_NUM)
#define I2C6_RX_DMA_DEV_2 (DT_PROP(DT_NODELABEL(i2c6), rx-dma-ext-channel) / DMA_CHANNEL_NUM)
#define I2C6_RX_DMA_CHN_2 (DT_PROP(DT_NODELABEL(i2c6), rx-dma-ext-channel) % DMA_CHANNEL_NUM)
#else
#define I2C6_RX_DMA_DEV I2C_NOT_USE_DMA
#define I2C6_RX_DMA_CHN I2C_NOT_USE_DMA
#define I2C6_RX_DMA_DEV_2 I2C_NOT_USE_DMA
#define I2C6_RX_DMA_CHN_2 I2C_NOT_USE_DMA
#endif
CREATE_I2C_INSTANCE(6);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c7), okay)
#if DT_NODE_HAS_PROP(DT_NODELABEL(i2c7), tx-dma-channel)
#define I2C7_TX_DMA_DEV (DT_PROP(DT_NODELABEL(i2c7), tx-dma-channel) / DMA_CHANNEL_NUM)
#define I2C7_TX_DMA_CHN (DT_PROP(DT_NODELABEL(i2c7), tx-dma-channel) % DMA_CHANNEL_NUM)
#else
#define I2C7_TX_DMA_DEV I2C_NOT_USE_DMA
#define I2C7_TX_DMA_CHN I2C_NOT_USE_DMA
#endif
#if (DT_NODE_HAS_PROP(DT_NODELABEL(i2c7), rx-dma-channel) && \
	DT_NODE_HAS_PROP(DT_NODELABEL(i2c7), rx-dma-ext-channel))
#define I2C7_RX_DMA_DEV (DT_PROP(DT_NODELABEL(i2c7), rx-dma-channel) / DMA_CHANNEL_NUM)
#define I2C7_RX_DMA_CHN (DT_PROP(DT_NODELABEL(i2c7), rx-dma-channel) % DMA_CHANNEL_NUM)
#define I2C7_RX_DMA_DEV_2 (DT_PROP(DT_NODELABEL(i2c7), rx-dma-ext-channel) / DMA_CHANNEL_NUM)
#define I2C7_RX_DMA_CHN_2 (DT_PROP(DT_NODELABEL(i2c7), rx-dma-ext-channel) % DMA_CHANNEL_NUM)
#else
#define I2C7_RX_DMA_DEV I2C_NOT_USE_DMA
#define I2C7_RX_DMA_CHN I2C_NOT_USE_DMA
#define I2C7_RX_DMA_DEV_2 I2C_NOT_USE_DMA
#define I2C7_RX_DMA_CHN_2 I2C_NOT_USE_DMA
#endif
CREATE_I2C_INSTANCE(7);
#endif

