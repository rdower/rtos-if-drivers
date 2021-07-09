/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sedi.h"
#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spi_sedi);
#include "spi_context.h"
#include <errno.h>
#include <kernel.h>
#include <drivers/spi.h>

#define DT_DRV_COMPAT intel_pse_spi

#define SPI_NOT_USE_DMA (-1)

typedef void (*spi_irq_config)(void);

struct spi_sedi_config {
	sedi_spi_t spi_device;
	spi_irq_config irq_config;
};

struct spi_sedi_data {
	struct spi_context ctx;
	sedi_spi_t spi_device;
	bool tx_data_updated;
	bool rx_data_updated;
	uint32_t tx_dummy_len;
	uint32_t rx_dummy_len;
	bool is_locked;
	int tx_dma_instance;
	int rx_dma_instance;
	int tx_channel;
	int rx_channel;
#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
	uint32_t device_power_state;
#endif
};

static int spi_sedi_configure(const struct device *dev,
			      const struct spi_config *config)
{
	struct spi_sedi_data *data = dev->data;
	uint32_t word_size, cpol, cpha, loopback;

	word_size = SPI_WORD_SIZE_GET(config->operation);
	sedi_spi_control(data->spi_device, SEDI_SPI_IOCTL_DATA_WIDTH,
			 word_size);

	/* CPOL and CPHA */
	cpol = SPI_MODE_GET(config->operation) & SPI_MODE_CPOL;
	cpha = SPI_MODE_GET(config->operation) & SPI_MODE_CPHA;

	if ((cpol == 0) && (cpha == 0)) {
		sedi_spi_control(data->spi_device, SEDI_SPI_IOCTL_CPOL0_CPHA0,
				 0);
	} else if ((cpol == 0) && (cpha == 1U)) {
		sedi_spi_control(data->spi_device, SEDI_SPI_IOCTL_CPOL0_CPHA1,
				 0);
	} else if ((cpol == 1) && (cpha == 0U)) {
		sedi_spi_control(data->spi_device, SEDI_SPI_IOCTL_CPOL1_CPHA0,
				 0);
	} else {
		sedi_spi_control(data->spi_device, SEDI_SPI_IOCTL_CPOL1_CPHA1,
				 0);
	}

	/* MSB and LSB */
	if (config->operation & SPI_TRANSFER_LSB) {
		sedi_spi_control(data->spi_device, SEDI_SPI_IOCTL_LSB, 0);
	}

	/* Set loopback */
	loopback = SPI_MODE_GET(config->operation) & SPI_MODE_LOOP;
	sedi_spi_control(data->spi_device, SEDI_SPI_IOCTL_LOOPBACK, loopback);

	/* Set baudrate */
	sedi_spi_control(data->spi_device, SEDI_SPI_IOCTL_SPEED_SET,
			 config->frequency);

	sedi_spi_control(data->spi_device, SEDI_SPI_IOCTL_CS_HW, config->slave);

	/* Set LOCK_ON */
	if ((config->operation) & SPI_LOCK_ON) {
		data->is_locked = true;
	}

	data->ctx.config = config;
	spi_context_cs_configure(&data->ctx);

	return 0;
}

static int transceive(const struct device *dev, const struct spi_config *config,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs, bool asynchronous,
		      struct k_poll_signal *signal)
{
	const struct spi_sedi_config *info = dev->config;
	struct spi_sedi_data *spi = dev->data;
	struct spi_context *ctx = &spi->ctx;
	int ret;
	uint32_t transfer_bytes = 0;
	uint8_t *data_out = NULL, *data_in = NULL;
	uint32_t i, dummy_len = 0;
	const struct spi_buf *buf;
	bool is_multibufs = false;

	spi_context_lock(&spi->ctx, asynchronous, signal, config);

	/* If device locked, will return error, need to release first */
	if ((spi->is_locked) && (!spi_context_configured(&spi->ctx, config))) {
		ret = -EIO;
		goto out;
	}

	/* Power up use default setting */
	ret = sedi_spi_set_power(info->spi_device, SEDI_POWER_FULL);
	if (ret) {
		goto out;
	}

	/* If need to configure, re-configure */
	if (config != NULL) {
		spi_sedi_configure(dev, config);
	}

	spi->tx_data_updated = false;
	spi->rx_data_updated = false;
	/* Set buffers info */
	spi_context_buffers_setup(&spi->ctx, tx_bufs, rx_bufs, 1);

	/* If it is multi-bufsets, if yes, even users set DMA transfer enable,
	 * use interrupt instead. Reason is DW SPI have no feature to configure
	 * cs pin manually, DMA operation cannot support multi buffer set.
	 */
	if ((ctx->tx_count > 1) || (ctx->rx_count > 1)) {
		is_multibufs = true;
	}

	if (ctx->tx_count > ctx->rx_count) {
		spi->tx_dummy_len = 0;
		for (i = ctx->rx_count; i < ctx->tx_count; i++) {
			buf = ctx->current_tx + i;
			dummy_len += buf->len;
		}
		spi->rx_dummy_len = dummy_len;
	} else if (ctx->tx_count < ctx->rx_count) {
		spi->rx_dummy_len = 0;
		for (i = ctx->tx_count; i < ctx->rx_count; i++) {
			buf = ctx->current_rx + i;
			dummy_len += buf->len;
		}
		spi->tx_dummy_len = dummy_len;
	} else {
		spi->tx_dummy_len = 0;
		spi->rx_dummy_len = 0;
	}

	if ((ctx->tx_len == 0) && (ctx->rx_len == 0)) {
		spi_context_cs_control(&spi->ctx, false);
		spi_context_complete(&spi->ctx, 0);
		return 0;
	}

	/* For multiple buffers, using continuous mode */
	sedi_spi_control(spi->spi_device, SEDI_SPI_IOCTL_BUFFER_SETS, 1);

	if (ctx->tx_len == 0) {
		/* rx only, nothing to tx */
		data_out = NULL;
		data_in = (uint8_t *)ctx->rx_buf;
		transfer_bytes = ctx->rx_len;
		spi->tx_dummy_len -= transfer_bytes;
	} else if (ctx->rx_len == 0) {
		/* tx only, nothing to rx */
		data_out = (uint8_t *)ctx->tx_buf;
		data_in = NULL;
		transfer_bytes = ctx->tx_len;
		spi->rx_dummy_len -= transfer_bytes;
	} else if (ctx->tx_len == ctx->rx_len) {
		/* rx and tx are the same length */
		data_out = (uint8_t *)ctx->tx_buf;
		data_in = (uint8_t *)ctx->rx_buf;
		transfer_bytes = ctx->tx_len;
	} else if (ctx->tx_len > ctx->rx_len) {
		/* Break up the tx into multiple transfers so we don't have to
		 * rx into a longer intermediate buffer. Leave chip select
		 * active between transfers.
		 */
		data_out = (uint8_t *)ctx->tx_buf;
		data_in = ctx->rx_buf;
		transfer_bytes = ctx->rx_len;
	} else {
		/* Break up the rx into multiple transfers so we don't have to
		 * tx from a longer intermediate buffer. Leave chip select
		 * active between transfers.
		 */
		data_out = (uint8_t *)ctx->tx_buf;
		data_in = ctx->rx_buf;
		transfer_bytes = ctx->tx_len;
	}

	spi_context_cs_control(&spi->ctx, true);

	device_busy_set(dev);
#if CONFIG_DMA_SEDI
	if ((spi->tx_dma_instance >= 0) && (spi->tx_channel >= 0) &&
	    (spi->rx_dma_instance >= 0) && (spi->rx_channel >= 0) &&
	    (is_multibufs == false)) {

		ret = sedi_spi_dma_transfer(info->spi_device,
					    spi->tx_dma_instance,
					    spi->tx_channel, data_out,
					    spi->rx_dma_instance,
					    spi->rx_channel, data_in,
					    transfer_bytes);
	} else {
#endif
		ret = sedi_spi_transfer(info->spi_device, data_out, data_in,
					transfer_bytes);
#if CONFIG_DMA_SEDI
	}
#endif

	if (ret != SEDI_DRIVER_OK) {
		goto out;
	}

	ret = spi_context_wait_for_completion(&spi->ctx);
out:
	spi_context_release(&spi->ctx, ret);
	device_busy_clear(dev);

	return ret;
}

static int spi_sedi_transceive(const struct device *dev,
			       const struct spi_config *config,
			       const struct spi_buf_set *tx_bufs,
			       const struct spi_buf_set *rx_bufs)
{
	return transceive(dev, config, tx_bufs, rx_bufs, false, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_sedi_transceive_async(struct device *dev,
				     const struct spi_config *config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs,
				     struct k_poll_signal *async)
{
	return transceive(dev, config, tx_bufs, rx_bufs, true, async);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_sedi_release(const struct device *dev,
			    const struct spi_config *config)
{
	struct spi_sedi_data *spi = dev->data;

	if (!spi_context_configured(&spi->ctx, config)) {
		return -EINVAL;
	}

	/* Lock status to false */
	spi->is_locked = false;
	spi_context_unlock_unconditionally(&spi->ctx);

	return 0;
}

extern void spi_isr(sedi_spi_t device);

void spi_sedi_callback(uint32_t event, void *param)
{
	struct device *dev = (struct device *)param;
	struct spi_sedi_data *spi = dev->data;
	struct spi_context *ctx = &spi->ctx;
	int error;

	/* Update the semaphore of spi context */
	if (event == SEDI_SPI_EVENT_DATA_LOST) {
		error = -EIO;
	} else {
		error = 0;
	}

	if ((event == SEDI_SPI_EVENT_COMPLETE) ||
	    (event == SEDI_SPI_EVENT_DATA_LOST)) {
		spi_context_cs_control(&spi->ctx, false);
		spi_context_complete(&spi->ctx, error);
	} else if (event == SEDI_SPI_EVENT_TX_FINISHED) {
		spi_context_update_tx(ctx, 1, ctx->tx_len);
		if (ctx->tx_len != 0) {
			sedi_spi_update_tx_buf(spi->spi_device, ctx->tx_buf,
					       ctx->tx_len);
			if ((ctx->rx_len == 0) &&
			    (spi->rx_data_updated == false)) {
				/* Update rx length if always no rx */
				sedi_spi_update_rx_buf(spi->spi_device, NULL,
						       spi->rx_dummy_len);
				spi->rx_data_updated = true;
			}
		} else if (spi->tx_data_updated == false) {
			sedi_spi_update_tx_buf(spi->spi_device, NULL,
					       spi->tx_dummy_len);
			spi->tx_data_updated = true;
		}
	} else if (event == SEDI_SPI_EVENT_RX_FINISHED) {
		spi_context_update_rx(ctx, 1, ctx->rx_len);
		if (ctx->rx_len != 0) {
			sedi_spi_update_rx_buf(spi->spi_device, ctx->rx_buf,
					       ctx->rx_len);
		}
	}
}

static const struct spi_driver_api sedi_spi_api = {
	.transceive = spi_sedi_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_sedi_transceive_async,
#endif  /* CONFIG_SPI_ASYNC */
	.release = spi_sedi_release,
};

int spi_sedi_init(const struct device *dev)
{
	const struct spi_sedi_config *info = dev->config;
	struct spi_sedi_data *spi = dev->data;
	int ret;

	ret = sedi_spi_init(info->spi_device, spi_sedi_callback, (void *)dev);
	if (ret != SEDI_DRIVER_OK) {
		return -ENODEV;
	}

	/* Init and connect IRQ */
	(info->irq_config)();

	spi_context_unlock_unconditionally(&spi->ctx);

	/* Lock state to false */
	spi->is_locked = false;

	return 0;
}

#define CREATE_SEDI_SPI_INSTANCE(num)				       \
	static void spi_##num##_irq_init(void)			       \
	{							       \
		IRQ_CONNECT(DT_INST_IRQN(num),			       \
			    DT_INST_IRQ(num, priority),		       \
			    spi_isr, num, 0);			       \
		irq_enable(DT_INST_IRQN(num));			       \
	}							       \
	static struct spi_sedi_data spi_##num##_data = {	       \
		SPI_CONTEXT_INIT_LOCK(spi_##num##_data, ctx),	       \
		SPI_CONTEXT_INIT_SYNC(spi_##num##_data, ctx),	       \
		.spi_device = num,				       \
		.tx_dma_instance = SPI##num##_TX_DMA_DEV,	       \
		.rx_dma_instance = SPI##num##_RX_DMA_DEV,	       \
		.tx_channel = SPI##num##_TX_DMA_CHN,		       \
		.rx_channel = SPI##num##_RX_DMA_CHN,		       \
	};							       \
	static struct spi_sedi_config spi_##num##_config = {	       \
		.spi_device = num, .irq_config = spi_##num##_irq_init  \
	};							       \
	DEVICE_DEFINE(spi_sedi_##num, "SPI_" # num, spi_sedi_init,     \
		      spi_sedi_device_ctrl,			       \
		      &spi_##num##_data, &spi_##num##_config,	       \
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
		      &sedi_spi_api)

#ifdef CONFIG_DEVICE_POWER_MANAGEMENT

static void spi_sedi_set_power_state(const struct device *dev,
				     uint32_t power_state)
{
	struct spi_sedi_data *context = dev->data;

	context->device_power_state = power_state;
}

static uint32_t spi_sedi_get_power_state(const struct device *dev)
{
	struct spi_sedi_data *context = dev->data;

	return context->device_power_state;
}

static int spi_suspend_device(const struct device *dev)
{
	const struct spi_sedi_config *config = dev->config;

	if (device_busy_check(dev)) {
		return -EBUSY;
	}

	int ret = sedi_spi_set_power(config->spi_device, SEDI_POWER_SUSPEND);

	if (ret != SEDI_DRIVER_OK) {
		return -EIO;
	}

	spi_sedi_set_power_state(dev, PM_DEVICE_STATE_SUSPEND);

	return 0;
}

static int spi_resume_device_from_suspend(const struct device *dev)
{
	const struct spi_sedi_config *config = dev->config;
	int ret;

	ret = sedi_spi_set_power(config->spi_device, SEDI_POWER_FULL);
	if (ret != SEDI_DRIVER_OK) {
		return -EIO;
	}

	spi_sedi_set_power_state(dev, PM_DEVICE_STATE_ACTIVE);
	device_busy_clear(dev);

	return 0;
}

static int spi_set_device_low_power(const struct device *dev)
{
	const struct spi_sedi_config *config = dev->config;

	if (device_busy_check(dev)) {
		return -EBUSY;
	}

	int ret;

	ret = sedi_spi_set_power(config->spi_device, SEDI_POWER_LOW);
	if (ret != SEDI_DRIVER_OK) {
		return -EIO;
	}

	spi_sedi_set_power_state(dev, PM_DEVICE_STATE_LOW_POWER);
	return 0;
}

static int spi_set_device_force_suspend(const struct device *dev)
{
	const struct spi_sedi_config *config = dev->config;
	int ret;

	ret = sedi_spi_set_power(config->spi_device, SEDI_POWER_FORCE_SUSPEND);
	if (ret != SEDI_DRIVER_OK) {
		return -EIO;
	}
	spi_sedi_set_power_state(dev, PM_DEVICE_STATE_SUSPEND);
	return 0;
}

static int spi_sedi_device_ctrl(const struct device *dev, uint32_t ctrl_command,
				void *context, device_pm_cb cb, void *arg)
{
	int ret = 0;

	if (ctrl_command == DEVICE_PM_SET_POWER_STATE) {

		switch (*((uint32_t *)context)) {
		case PM_DEVICE_STATE_SUSPEND:
			ret = spi_suspend_device(dev);
			break;
		case PM_DEVICE_STATE_ACTIVE:
			ret = spi_resume_device_from_suspend(dev);
			break;
		case PM_DEVICE_STATE_LOW_POWER:
			ret = spi_set_device_low_power(dev);
			break;
		case PM_DEVICE_STATE_SUSPEND:
			ret = spi_set_device_force_suspend(dev);
			break;
		default:
			ret = -ENOTSUP;
		}
	} else if (ctrl_command == DEVICE_PM_GET_POWER_STATE) {
		*((uint32_t *)context) = spi_sedi_get_power_state(dev);
	}

	if (cb) {
		cb(dev, ret, context, arg);
	}

	return ret;
}

#endif /* CONFIG_DEVICE_POWER_MANAGEMENT */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi0), okay)
#if DT_NODE_HAS_PROP(DT_NODELABEL(spi0), tx-dma-channel)
#define SPI0_TX_DMA_DEV (DT_PROP(DT_NODELABEL(spi0), tx-dma-channel) / DMA_CHANNEL_NUM)
#define SPI0_TX_DMA_CHN (DT_PROP(DT_NODELABEL(spi0), tx-dma-channel) % DMA_CHANNEL_NUM)
#else
#define SPI0_TX_DMA_DEV SPI_NOT_USE_DMA
#define SPI0_TX_DMA_CHN SPI_NOT_USE_DMA
#endif
#if DT_NODE_HAS_PROP(DT_NODELABEL(spi0), rx-dma-channel)
#define SPI0_RX_DMA_DEV (DT_PROP(DT_NODELABEL(spi0), rx-dma-channel) / DMA_CHANNEL_NUM)
#define SPI0_RX_DMA_CHN (DT_PROP(DT_NODELABEL(spi0), rx-dma-channel) % DMA_CHANNEL_NUM)
#else
#define SPI0_RX_DMA_DEV SPI_NOT_USE_DMA
#define SPI0_RX_DMA_CHN SPI_NOT_USE_DMA
#endif
CREATE_SEDI_SPI_INSTANCE(0);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi1), okay)
#if DT_NODE_HAS_PROP(DT_NODELABEL(spi1), tx-dma-channel)
#define SPI1_TX_DMA_DEV (DT_PROP(DT_NODELABEL(spi1), tx-dma-channel) / DMA_CHANNEL_NUM)
#define SPI1_TX_DMA_CHN (DT_PROP(DT_NODELABEL(spi1), tx-dma-channel) % DMA_CHANNEL_NUM)
#else
#define SPI1_TX_DMA_DEV SPI_NOT_USE_DMA
#define SPI1_TX_DMA_CHN SPI_NOT_USE_DMA
#endif
#if DT_NODE_HAS_PROP(DT_NODELABEL(spi1), rx-dma-channel)
#define SPI1_RX_DMA_DEV (DT_PROP(DT_NODELABEL(spi1), rx-dma-channel) / DMA_CHANNEL_NUM)
#define SPI1_RX_DMA_CHN (DT_PROP(DT_NODELABEL(spi1), rx-dma-channel) % DMA_CHANNEL_NUM)
#else
#define SPI1_RX_DMA_DEV SPI_NOT_USE_DMA
#define SPI1_RX_DMA_CHN SPI_NOT_USE_DMA
#endif
CREATE_SEDI_SPI_INSTANCE(1);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi2), okay)
#if DT_NODE_HAS_PROP(DT_NODELABEL(spi2), tx-dma-channel)
#define SPI2_TX_DMA_DEV (DT_PROP(DT_NODELABEL(spi2), tx-dma-channel) / DMA_CHANNEL_NUM)
#define SPI2_TX_DMA_CHN (DT_PROP(DT_NODELABEL(spi2), tx-dma-channel) % DMA_CHANNEL_NUM)
#else
#define SPI2_TX_DMA_DEV SPI_NOT_USE_DMA
#define SPI2_TX_DMA_CHN SPI_NOT_USE_DMA
#endif
#if DT_NODE_HAS_PROP(DT_NODELABEL(spi2), rx-dma-channel)
#define SPI2_RX_DMA_DEV (DT_PROP(DT_NODELABEL(spi2), rx-dma-channel) / DMA_CHANNEL_NUM)
#define SPI2_RX_DMA_CHN (DT_PROP(DT_NODELABEL(spi2), rx-dma-channel) % DMA_CHANNEL_NUM)
#else
#define SPI2_RX_DMA_DEV SPI_NOT_USE_DMA
#define SPI2_RX_DMA_CHN SPI_NOT_USE_DMA
#endif
CREATE_SEDI_SPI_INSTANCE(2);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi3), okay)
#if DT_NODE_HAS_PROP(DT_NODELABEL(spi3), tx-dma-channel)
#define SPI3_TX_DMA_DEV (DT_PROP(DT_NODELABEL(spi3), tx-dma-channel) / DMA_CHANNEL_NUM)
#define SPI3_TX_DMA_CHN (DT_PROP(DT_NODELABEL(spi3), tx-dma-channel) % DMA_CHANNEL_NUM)
#else
#define SPI3_TX_DMA_DEV SPI_NOT_USE_DMA
#define SPI3_TX_DMA_CHN SPI_NOT_USE_DMA
#endif
#if DT_NODE_HAS_PROP(DT_NODELABEL(spi3), rx-dma-channel)
#define SPI3_RX_DMA_DEV (DT_PROP(DT_NODELABEL(spi3), rx-dma-channel) / DMA_CHANNEL_NUM)
#define SPI3_RX_DMA_CHN (DT_PROP(DT_NODELABEL(spi3), rx-dma-channel) % DMA_CHANNEL_NUM)
#else
#define SPI3_RX_DMA_DEV SPI_NOT_USE_DMA
#define SPI3_RX_DMA_CHN SPI_NOT_USE_DMA
#endif
CREATE_SEDI_SPI_INSTANCE(3);
#endif
