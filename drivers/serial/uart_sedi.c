/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <device.h>
#include <drivers/uart.h>
#include "driver/sedi_driver_uart.h"
#include "arch/arm/aarch32/cortex_m/cmsis.h"
#include "driver/sedi_driver_ipc.h"
#include "driver/sedi_driver_pm.h"
#include <logging/log.h>
LOG_MODULE_REGISTER(uart, LOG_LEVEL_ERR);

#define LINE_CONTROL SEDI_UART_LC_8N1
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_sedi_isr(void *arg);
static void uart_sedi_cb(const struct device *port);
#endif

#define DT_DRV_COMPAT intel_pse_uart

/* Helper macro to set flow control. */
#define UART_CONFIG_FLOW_CTRL_SET(n) \
	.hw_fc = DT_PROP(DT_NODELABEL(uart##n), hw_flow_control)

#ifdef CONFIG_UART_9_BIT
#define UART_NODE_ID_MASK (0xFF)
#define UART_NODE_ID_SET(n) \
	.node_id = CONFIG_UART_SEDI_##n##_RX_ADDR,
#else
#define UART_NODE_ID_SET(n)
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
/*  UART IRQ handler declaration.  */
#define  UART_IRQ_HANDLER_DECL(n) \
	static void irq_config_uart_##n(const struct device *dev)

/* Setting configuration function. */
#define UART_CONFIG_IRQ_HANDLER_SET(n) \
	.uart_irq_config_func = irq_config_uart_##n
#define UART_IRQ_HANDLER_DEFINE(n)					       \
	static void irq_config_uart_##n(const struct device *dev)	       \
	{								       \
		ARG_UNUSED(dev);					       \
		IRQ_CONNECT(DT_IRQ(DT_NODELABEL(uart##n), irq),		       \
			    DT_IRQ(DT_NODELABEL(			       \
					   uart##n), priority), uart_sedi_isr, \
			    DEVICE_GET(uart_##n), 0);			       \
		irq_enable(DT_IRQ(DT_NODELABEL(uart##n), irq));		       \
	}
#else /*CONFIG_UART_INTERRUPT_DRIVEN */
#define UART_IRQ_HANDLER_DECL(n)
#define UART_CONFIG_IRQ_HANDLER_SET(n) (0)

#define UART_IRQ_HANDLER_DEFINE(n)
#endif  /* !CONFIG_UART_INTERRUPT_DRIVEN */

/* Device init macro for UART instance. As multiple uart instances follow a
 * similar definition of data structures differing only in the instance
 * number.This macro makes adding instances simpler.
 */
#define UART_SEDI_DEVICE_INIT(n)					    \
	UART_IRQ_HANDLER_DECL(n);					    \
	static K_MUTEX_DEFINE(uart_##n##_mutex);			    \
	static K_SEM_DEFINE(uart_##n##_tx_sem, 1, 1);			    \
	static K_SEM_DEFINE(uart_##n##_rx_sem, 1, 1);			    \
	static K_SEM_DEFINE(uart_##n##_sync_read_sem, 0, 1);		    \
	static const struct uart_sedi_config_info config_info_##n = {	    \
		.instance = SEDI_UART_##n,				    \
		.baud_rate = DT_PROP(DT_NODELABEL(uart##n), current_speed), \
		UART_CONFIG_FLOW_CTRL_SET(n),				    \
		.line_ctrl = LINE_CONTROL,				    \
		.mutex = &uart_##n##_mutex,				    \
		.tx_sem = &uart_##n##_tx_sem,				    \
		.rx_sem = &uart_##n##_rx_sem,				    \
		.sync_read_sem = &uart_##n##_sync_read_sem,		    \
		UART_NODE_ID_SET(n)					    \
		UART_CONFIG_IRQ_HANDLER_SET(n)				    \
	};								    \
									    \
	static struct uart_sedi_drv_data drv_data_##n;			    \
	DEVICE_DEFINE(uart_##n, DT_LABEL(DT_NODELABEL(uart##n)),	    \
		      &uart_sedi_init,					    \
		      &uart_sedi_device_ctrl,				    \
		      &drv_data_##n, &config_info_##n,			    \
		      PRE_KERNEL_1,					    \
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &api);	    \
	UART_IRQ_HANDLER_DEFINE(n)


/* Convenient macro to get the controller instance. */
#define GET_CONTROLLER_INSTANCE(dev)		 \
	(((const struct uart_sedi_config_info *) \
	  dev->config)->instance)

/* Convenient macro to get tx semamphore */
#define GET_TX_SEM(dev)				 \
	(((const struct uart_sedi_config_info *) \
	  dev->config)->tx_sem)

/* Convenient macro to get rx sempahore */
#define GET_RX_SEM(dev)				 \
	(((const struct uart_sedi_config_info *) \
	  dev->config)->rx_sem)

/* Convenient macro to get sync_read sempahore */
#define GET_SYNC_READ_SEM(dev)			 \
	(((const struct uart_sedi_config_info *) \
	  dev->config)->sync_read_sem)

#define GET_MUTEX(dev)				 \
	(((const struct uart_sedi_config_info *) \
	  dev->config)->mutex)

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#ifdef CONFIG_UART_SEDI_USE_DMA
/* Ctrlr 0 used for tx. Channel for uart instance = instance number. */
#define UART_DMA_CONTROLLER_TX (0)
/* Ctrlr 1 used for rx. Channel for uart instance = instance number.  */
#define UART_DMA_CONTROLLER_RX (1)
static sedi_uart_dma_xfer_t dma_tx_xfer[SEDI_UART_NUM];
static sedi_uart_dma_xfer_t dma_rx_xfer[SEDI_UART_NUM];
#else
static sedi_uart_transfer_t xfer_tx[SEDI_UART_NUM];
static sedi_uart_transfer_t xfer_rx[SEDI_UART_NUM];
#endif
static sedi_uart_unsol_rx_t unsol_rx[SEDI_UART_NUM];
static sedi_uart_io_vec_xfer_t vec_xfer_tx[SEDI_UART_NUM];
static sedi_uart_io_vec_xfer_t vec_xfer_rx[SEDI_UART_NUM];
#endif

struct uart_sedi_config_info {
	/* Specifies the uart instance for configuration. */
	sedi_uart_t instance;

	/* Specifies the baudrate for the uart instance. */
	uint32_t baud_rate;

	/* Specifies the port line control settings */
	sedi_uart_lc_t line_ctrl;

	struct k_mutex *mutex;
	struct k_sem *tx_sem;
	struct k_sem *rx_sem;
	struct k_sem *sync_read_sem;
	/* Enable / disable hardware flow control for UART. */

	bool hw_fc;

	/* UART irq configuration function when supporting interrupt
	 * mode.
	 */
	uart_irq_config_func_t uart_irq_config_func;

#ifdef CONFIG_UART_9_BIT
	/* Rx address of  this node when in 9-bit mode */
	uint8_t node_id;
#endif

};

static int uart_sedi_init(const struct device *dev);

struct uart_sedi_drv_data {
	uart_irq_callback_user_data_t user_cb;
	uart_xfer_cb_t active_tx_xfer_cb;
	uart_xfer_cb_t active_rx_xfer_cb;
	uart_unsol_rx_cb_t unsol_rx_usr_cb;
	void *unsol_rx_usr_cb_param;
	uint32_t sync_rx_len;
	uint32_t sync_rx_status;
	void *user_data;
	void *usr_rx_buff;
	uint32_t usr_rx_size;

	/* PM callback for freq. change notification */
	sedi_pm_callback_config_t pm_rst;

#ifdef CONFIG_PM_DEVICE
	uint32_t device_power_state;
#endif
	uint8_t iir_cache;
	int32_t busy_count;
};

static void uart_busy_set(const struct device *dev)
{

#ifdef CONFIG_PM_DEVICE
	struct uart_sedi_drv_data *context = dev->data;

	atomic_inc((atomic_t *)&context->busy_count);
	if (pm_device_is_busy(dev) == 0) {
		pm_device_busy_set(dev);
	}
#else
	ARG_UNUSED(dev);
#endif

}

static void uart_busy_clear(const struct device *dev)
{

#ifdef CONFIG_PM_DEVICE
	struct uart_sedi_drv_data *context = dev->data;
	uint32_t key;

	key = irq_lock();
	if (context->busy_count > 0) {
		context->busy_count--;
		if (context->busy_count ==  0) {
			pm_device_busy_clear(dev);
		}
	}
	irq_unlock(key);
#else
	ARG_UNUSED(dev);
#endif
}

#ifdef CONFIG_PM_DEVICE
static void uart_sedi_set_power_state(const struct device *dev,
				      enum pm_device_state power_state)
{
	struct uart_sedi_drv_data *context = dev->data;

	context->device_power_state = power_state;
}

static uint32_t uart_sedi_get_power_state(const struct device *dev)
{
	struct uart_sedi_drv_data *context = dev->data;

	return context->device_power_state;
}

static int uart_suspend_device(const struct device *dev)
{
	const struct uart_sedi_config_info *config = dev->config;

	if (pm_device_is_busy(dev)) {
		return -EBUSY;
	}

	int ret = sedi_uart_set_power(config->instance, SEDI_POWER_SUSPEND);

	if (ret != SEDI_DRIVER_OK) {
		return -EIO;
	}

	uart_sedi_set_power_state(dev, PM_DEVICE_STATE_SUSPEND);

	return 0;
}

static int uart_resume_device_from_suspend(const struct device *dev)
{
	const struct uart_sedi_config_info *config = dev->config;
	int ret;

	ret = sedi_uart_set_power(config->instance, SEDI_POWER_FULL);
	if (ret != SEDI_DRIVER_OK) {
		return -EIO;
	}

	uart_sedi_set_power_state(dev, PM_DEVICE_STATE_ACTIVE);

	return 0;
}

static int uart_set_device_low_power(const struct device *dev)
{
	const struct uart_sedi_config_info *config = dev->config;

	if (pm_device_is_busy(dev)) {
		return -EBUSY;
	}

	int ret;

	ret = sedi_uart_set_power(config->instance, SEDI_POWER_LOW);
	if (ret != SEDI_DRIVER_OK) {
		return -EIO;
	}

	uart_sedi_set_power_state(dev, PM_DEVICE_STATE_LOW_POWER);
	return 0;
}

static int uart_set_device_force_suspend(const struct device *dev)
{
	const struct uart_sedi_config_info *config = dev->config;
	int ret;

	ret = sedi_uart_set_power(config->instance, SEDI_POWER_FORCE_SUSPEND);
	if (ret != SEDI_DRIVER_OK) {
		return -EIO;
	}
	uart_sedi_set_power_state(dev, PM_DEVICE_STATE_FORCE_SUSPEND);
	return 0;
}

static int(uart_sedi_device_ctrl) (const struct device *dev,
				   uint32_t ctrl_command,
				   enum pm_device_state *state) {
	int ret = 0;

	if (ctrl_command == PM_DEVICE_STATE_SET) {

		switch (*state) {
		case PM_DEVICE_STATE_SUSPEND:
			ret = uart_suspend_device(dev);
			break;
		case PM_DEVICE_STATE_ACTIVE:
			ret = uart_resume_device_from_suspend(dev);
			break;
		case PM_DEVICE_STATE_LOW_POWER:
			ret = uart_set_device_low_power(dev);
			break;
		case PM_DEVICE_STATE_FORCE_SUSPEND:
			ret = uart_set_device_force_suspend(dev);
			break;
		default:
			ret = -ENOTSUP;
		}
	} else if (ctrl_command == PM_DEVICE_STATE_GET) {
		*(state) = uart_sedi_get_power_state(dev);
	}

	return ret;
}

#endif /* CONFIG_PM_DEVICE */

#ifdef CONFIG_UART_ASYNC_API

int uart_sedi_callback_set(const struct device *dev,
			   uart_callback_t callback,
			   void *user_data)
{
	return -ENOTSUP;
}

int uart_sedi_tx(const struct device *dev,
		 const uint8_t *buf,
		 size_t len,
		 int32_t timeout)
{
	return -ENOTSUP;
}
int uart_sedi_tx_abort(const struct device *dev)
{
	return -ENOTSUP;
}

int uart_sedi_rx_enable(const struct device *dev,
			uint8_t *buf,
			size_t len,
			int32_t timeout)
{
	return -ENOTSUP;
}

int uart_sedi_rx_buf_rsp(const struct device *dev, uint8_t *buf, size_t len)
{
	return -ENOTSUP;
}

int uart_sedi_rx_disable(const struct device *dev)
{
	return -ENOTSUP;
}

#endif

static int uart_sedi_poll_in(const struct device *dev, unsigned char *data)
{
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);
	uint32_t status;
	int ret = 0;

	sedi_uart_get_status(instance, (uint32_t *) &status);

	/* In order to check if there is any data to read from UART
	 * controller we should check if the SEDI_UART_RX_BUSY bit from
	 * 'status' is not set. This bit is set only if there is any
	 * pending character to read.
	 */
	if (!(status & SEDI_UART_RX_BUSY)) {
		ret = -1;
	} else {
		if (sedi_uart_read(instance, data, (uint32_t *)&status)) {
			ret = -1;
		}
	}
	return ret;
}

static void uart_sedi_poll_out(const struct device *dev,
			       unsigned char data)
{
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	sedi_uart_write(instance, data);
}

static int get_xfer_error(int bsp_err)
{
	int err;

	switch (bsp_err) {
	case SEDI_DRIVER_OK:
		err = 0;
		break;
	case SEDI_USART_ERROR_CANCELED:
		err = -ECANCELED;
		break;
	case SEDI_DRIVER_ERROR:
		err = -EIO;
		break;
	case SEDI_DRIVER_ERROR_PARAMETER:
		err = -EINVAL;
		break;
	case SEDI_DRIVER_ERROR_UNSUPPORTED:
		err = -ENOTSUP;
		break;
	default:
		err = -EFAULT;
	}
	return err;
}

static uint32_t uart_sedi_decode_line_err(uint32_t bsp_line_err)
{
	uint32_t zephyr_line_err = 0;

	if (bsp_line_err  &  SEDI_UART_RX_OE) {
		zephyr_line_err = UART_ERROR_OVERRUN;
	}

	if (bsp_line_err & SEDI_UART_RX_PE) {
		zephyr_line_err = UART_ERROR_PARITY;
	}

	if (bsp_line_err  & SEDI_UART_RX_FE) {
		zephyr_line_err = UART_ERROR_FRAMING;
	}

	if (bsp_line_err & SEDI_UART_RX_BI) {
		zephyr_line_err = UART_BREAK;
	}

	return zephyr_line_err;
}

static int uart_sedi_read_buffer_polled(const struct device *dev, uint8_t *buff,
					int len,  uint32_t *line_status)
{
	__ASSERT(buff != NULL, "");
	__ASSERT(len != 0, "");
	__ASSERT(line_status != NULL, "");

	uint32_t comp_len, bsp_line_err;
	int ret;

	if (k_sem_take(GET_RX_SEM(dev), K_NO_WAIT)) {
		LOG_ERR("Failed to acquire RX semaphore");
		return -EBUSY;
	}

	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	uart_busy_set(dev);
#ifdef CONFIG_UART_SEDI_USE_DMA
	ret = sedi_uart_dma_read_polled(instance, UART_DMA_CONTROLLER_RX,
					instance, buff, len, &bsp_line_err);
#if !defined(CONFIG_CACHE_DISABLE)
	SCB_InvalidateDCache_by_Addr((uint32_t *)buff,
				     len);
#endif
	comp_len = len;
#else
	ret = sedi_uart_read_buffer(instance,
				    buff,
				    (uint32_t)len,
				    (uint32_t *)&comp_len,
				    (uint32_t *)&bsp_line_err);
#endif

	if (ret != SEDI_DRIVER_OK) {
		ret = get_xfer_error(ret);
		LOG_ERR("Failed to read buffer, ret:%d", ret);
	} else {
		ret = comp_len;
	}

	*line_status = uart_sedi_decode_line_err(bsp_line_err);
	uart_busy_clear(dev);
	k_sem_give(GET_RX_SEM(dev));
	return ret;

}

static int uart_sedi_write_buffer_polled(const struct device *dev,
					 uint8_t *buff,
					 int len)
{
	__ASSERT(buff != NULL, "");
	__ASSERT(len != 0, "");

	int ret;
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	if (k_sem_take(GET_TX_SEM(dev), K_NO_WAIT)) {
		return -EBUSY;
	}

	uart_busy_set(dev);
#ifdef CONFIG_UART_SEDI_USE_DMA
#if !defined(CONFIG_CACHE_DISABLE)
	SCB_CleanDCache_by_Addr((uint32_t *)buff, len);
#endif
	ret = sedi_uart_dma_write_polled(instance, UART_DMA_CONTROLLER_TX,
					 instance, buff, len);
	if (ret != SEDI_DRIVER_OK) {
		ret = get_xfer_error(ret);
	} else {
		ret = len;
	}
#else
	ret = sedi_uart_write_buffer(instance, buff, len);
	__ASSERT(ret == SEDI_DRIVER_OK, "");
	ret = len;
#endif
	uart_busy_clear(dev);
	k_sem_give(GET_TX_SEM(dev));
	return ret;
}

static int uart_sedi_err_check(const struct device *dev)
{
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);
	uint32_t status;
	int ret_status = 0;

	sedi_uart_get_status(instance, (uint32_t *const)&status);
	if (status &  SEDI_UART_RX_OE) {
		ret_status = UART_ERROR_OVERRUN;
	}

	if (status & SEDI_UART_RX_PE) {
		ret_status = UART_ERROR_PARITY;
	}

	if (status & SEDI_UART_RX_FE) {
		ret_status = UART_ERROR_FRAMING;
	}

	return ret_status;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int uart_sedi_fifo_fill(const struct device *dev, const uint8_t *tx_data,
			       int size)
{
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	return sedi_uart_fifo_fill(instance, tx_data, size);
}

static int uart_sedi_fifo_read(const struct device *dev, uint8_t *rx_data,
			       const int size)
{
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	return sedi_uart_fifo_read(instance, rx_data, size);
}

static void uart_sedi_irq_tx_enable(const struct device *dev)
{
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	uart_busy_set(dev);
	sedi_uart_irq_tx_enable(instance);
}

static void uart_sedi_irq_tx_disable(const struct device *dev)
{
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	uart_busy_clear(dev);
	sedi_uart_irq_tx_disable(instance);
}

static int uart_sedi_irq_tx_ready(const struct device *dev)
{
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	return sedi_uart_irq_tx_ready(instance);
}

static int uart_sedi_irq_tx_complete(const struct device *dev)
{
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	return sedi_uart_is_tx_complete(instance);
}

static void uart_sedi_irq_rx_enable(const struct device *dev)
{
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	uart_busy_set(dev);
	sedi_uart_irq_rx_enable(instance);
}

static void uart_sedi_irq_rx_disable(const struct device *dev)
{
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	uart_busy_clear(dev);
	sedi_uart_irq_rx_disable(instance);
}

static int uart_sedi_irq_rx_ready(const struct device *dev)
{
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	return sedi_uart_is_irq_rx_ready(instance);
}

static void uart_sedi_irq_err_enable(const struct device *dev)
{
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	sedi_uart_irq_err_enable(instance);
}

static void uart_sedi_irq_err_disable(const struct device *dev)
{
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	sedi_uart_irq_err_disable(instance);
}

static int uart_sedi_irq_is_pending(const struct device *dev)
{

	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	return sedi_uart_is_irq_pending(instance);
}

static int uart_sedi_irq_update(const struct device *dev)
{
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	sedi_uart_update_irq_cache(instance);
	return 1;
}

static void uart_sedi_irq_callback_set(const struct device *dev,
				       uart_irq_callback_user_data_t cb,
				       void *user_data)
{
	struct uart_sedi_drv_data *drv_data = dev->data;

	drv_data->user_cb = cb;
	drv_data->user_data = user_data;

}

static void uart_sedi_isr(void *arg)
{
	const struct device *dev = arg;
	struct uart_sedi_drv_data *drv_data = dev->data;

	if (drv_data->user_cb) {
		drv_data->user_cb(dev, drv_data->user_data);
	} else {
		uart_sedi_cb(dev);
	}
}


/* Called from generic callback of zephyr , set by set_cb. */
static void uart_sedi_cb(const struct device *port)
{
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(port);

	sedi_uart_isr_handler(instance);
}

static void uart_irq_xfer_tx_common_cb(void *data, int err, uint32_t status,
				       uint32_t len)
{
	const struct device *dev = data;
	struct uart_sedi_drv_data *drv_data = dev->data;
	int driver_err;

	if (drv_data->active_tx_xfer_cb) {
		driver_err = get_xfer_error(err);
		drv_data->active_tx_xfer_cb(dev, err, 0, len);
		drv_data->active_tx_xfer_cb = NULL;
	}
	uart_busy_clear(dev);
	k_sem_give(GET_TX_SEM(dev));
}

static void uart_irq_xfer_rx_common_cb(void *data, int err, uint32_t status,
				       uint32_t len)
{

	const struct device *dev = data;
	struct uart_sedi_drv_data *drv_data = dev->data;
	int driver_err;
	uint32_t rx_err;

	if (drv_data->active_rx_xfer_cb) {
		driver_err = get_xfer_error(err);
		rx_err = uart_sedi_decode_line_err(status);
#if !defined(CONFIG_CACHE_DISABLE)
		SCB_InvalidateDCache_by_Addr((uint32_t *)drv_data->usr_rx_buff,
					     drv_data->usr_rx_size);
#endif
		drv_data->active_rx_xfer_cb(dev, driver_err, rx_err, len);
		drv_data->active_rx_xfer_cb = NULL;
	}
	uart_busy_clear(dev);
	k_sem_give(GET_RX_SEM(dev));
}

static void uart_irq_xfer_rx_timed_cb(void *data, int err, uint32_t status,
				      uint32_t len)
{
	const struct device *dev = data;
	struct uart_sedi_drv_data *drv_data = dev->data;

	drv_data->sync_rx_len = len;
	drv_data->sync_rx_status = status;
	if (err != SEDI_USART_ERROR_CANCELED) {
		k_sem_give(GET_SYNC_READ_SEM(dev));
	}
}

static int uart_sedi_write_buffer_async(const struct device *dev, uint8_t *buff,
					uint32_t len, uart_xfer_cb_t xfer_cb)
{
	int ret;

	__ASSERT(buff != NULL, "");
	__ASSERT(len != 0, "");

	if (k_sem_take(GET_TX_SEM(dev), K_NO_WAIT)) {
		return -EBUSY;
	}

	struct uart_sedi_drv_data *drv_data = dev->data;

	drv_data->active_tx_xfer_cb = xfer_cb;
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	uart_busy_set(dev);

#if CONFIG_UART_SEDI_USE_DMA
	dma_tx_xfer[instance].dma_dev = UART_DMA_CONTROLLER_TX;
	dma_tx_xfer[instance].channel = instance;
	dma_tx_xfer[instance].data = buff;
	dma_tx_xfer[instance].len = len;
	dma_tx_xfer[instance].callback = uart_irq_xfer_tx_common_cb;
	dma_tx_xfer[instance].cb_param = (void *)dev;
#if !defined(CONFIG_CACHE_DISABLE)
	SCB_CleanDCache_by_Addr((uint32_t *)buff, len);
#endif

	ret = sedi_uart_dma_write_async(instance, &dma_tx_xfer[instance]);
#else
	xfer_tx[instance].data = buff;
	xfer_tx[instance].data_len = len;
	xfer_tx[instance].callback = uart_irq_xfer_tx_common_cb;
	xfer_tx[instance].callback_data = (void *)dev;
	ret = sedi_uart_write_async(instance, &xfer_tx[instance]);
#endif
	if (ret != SEDI_DRIVER_OK) {
		ret = get_xfer_error(ret);
		uart_busy_clear(dev);
		k_sem_give(GET_TX_SEM(dev));
		return ret;
	}

	return 0;
}

static int uart_sedi_read_buffer_async(const struct device *dev, uint8_t *buff,
				       uint32_t len, uart_xfer_cb_t xfer_cb)
{
	int ret;

	__ASSERT(buff != NULL, "");
	__ASSERT(len != 0, "");

	if (k_sem_take(GET_RX_SEM(dev), K_NO_WAIT)) {
		LOG_ERR("Failed to acquire RX semaphore");
		return -EBUSY;
	}
	struct uart_sedi_drv_data *drv_data = dev->data;

	drv_data->active_rx_xfer_cb = xfer_cb;
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	uart_busy_set(dev);
#if CONFIG_UART_SEDI_USE_DMA
	dma_rx_xfer[instance].dma_dev = UART_DMA_CONTROLLER_RX;
	dma_rx_xfer[instance].channel = instance;
	dma_rx_xfer[instance].data = buff;
	dma_rx_xfer[instance].len = len;
	dma_rx_xfer[instance].callback = uart_irq_xfer_rx_common_cb;
	dma_rx_xfer[instance].cb_param = (void *)dev;
	drv_data->usr_rx_buff = (void *)buff;
	drv_data->usr_rx_size = len;
	ret = sedi_uart_dma_read_async(instance, &dma_rx_xfer[instance]);
#else
	xfer_rx[instance].data = buff;
	xfer_rx[instance].data_len = len;
	xfer_rx[instance].callback = uart_irq_xfer_rx_common_cb;
	xfer_rx[instance].callback_data = (void *)dev;
	ret = sedi_uart_read_async(instance, &xfer_rx[instance]);
#endif
	if (ret != SEDI_DRIVER_OK) {
		uart_busy_clear(dev);
		k_sem_give(GET_RX_SEM(dev));
		ret = get_xfer_error(ret);
		LOG_ERR("Read async failed. ret:%d", ret);
		return ret;
	}

	return 0;
}

static int uart_sedi_read_buffer_sync(const struct device *dev,
				      uint8_t *buff,
				      int32_t len,
				      uint32_t timeout,
				      uint32_t *status)
{

	__ASSERT(buff != NULL, "");
	__ASSERT(len != 0, "");
	__ASSERT(status != NULL, "");

	int ret;
	*status = 0;

	if (k_sem_take(GET_RX_SEM(dev), K_NO_WAIT)) {
		LOG_ERR("Failed to acquire RX semaphore");
		return -EBUSY;
	}
	struct uart_sedi_drv_data *drv_data = dev->data;

	drv_data->active_rx_xfer_cb = NULL;
	drv_data->sync_rx_len = 0;
	drv_data->sync_rx_status = -1;
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	uart_busy_set(dev);

#if CONFIG_UART_SEDI_USE_DMA
	dma_rx_xfer[instance].dma_dev = UART_DMA_CONTROLLER_RX;
	dma_rx_xfer[instance].channel = instance;
	dma_rx_xfer[instance].data = buff;
	dma_rx_xfer[instance].len = len;
	dma_rx_xfer[instance].callback = uart_irq_xfer_rx_timed_cb;
	dma_rx_xfer[instance].cb_param = (void *)dev;
	ret = sedi_uart_dma_read_async(instance, &dma_rx_xfer[instance]);
#else
	xfer_rx[instance].data = buff;
	xfer_rx[instance].data_len = len;
	xfer_rx[instance].callback = uart_irq_xfer_rx_timed_cb;
	xfer_rx[instance].callback_data = (void *)dev;
	ret = sedi_uart_read_async(instance, &xfer_rx[instance]);
#endif

	if (ret != SEDI_DRIVER_OK) {
		uart_busy_clear(dev);
		k_sem_give(GET_RX_SEM(dev));
		ret = get_xfer_error(ret);
		LOG_ERR("Read sync failed. ret:%d", ret);
		return ret;
	}
	if (k_sem_take(GET_SYNC_READ_SEM(dev), K_MSEC(timeout))) {
		/* Read timed out. */

#if CONFIG_UART_SEDI_USE_DMA
		ret = sedi_uart_dma_read_terminate(instance);
#else
		ret = sedi_uart_async_read_terminate(instance);
#endif
		if (ret != SEDI_DRIVER_OK) {
			uart_busy_clear(dev);
			k_sem_give(GET_RX_SEM(dev));
			ret = get_xfer_error(ret);
			LOG_ERR("Read sync failed. ret:%d", ret);
			return ret;
		}
		uart_busy_clear(dev);
		k_sem_give(GET_RX_SEM(dev));
		return -ETIMEDOUT;
	}

	*status = uart_sedi_decode_line_err(drv_data->sync_rx_status);
	if (drv_data->sync_rx_status) {
		uart_busy_clear(dev);
		k_sem_give(GET_RX_SEM(dev));
		LOG_ERR("Failed to decode line error");
		return -EIO;
	}

#if CONFIG_UART_SEDI_USE_DMA
#if !defined(CONFIG_CACHE_DISABLE)
	SCB_InvalidateDCache_by_Addr((uint32_t *)buff, len);
#endif
#endif

	uart_busy_clear(dev);
	k_sem_give(GET_RX_SEM(dev));
	return drv_data->sync_rx_len;

}

static void unsol_rx_cb(void *data, int bsp_err, uint32_t line_status, int len)
{
	const struct device *dev = data;
	struct uart_sedi_drv_data *drv_data = dev->data;
	void *usr_param;
	uint32_t line_err;
	int err;

	err = get_xfer_error(bsp_err);
	usr_param = drv_data->unsol_rx_usr_cb_param;
	line_err = uart_sedi_decode_line_err(line_status);
	drv_data->unsol_rx_usr_cb(dev, usr_param, err, line_err, len);

}

static int uart_sedi_enable_unsol_receive(const struct device *dev,
					  uint8_t *buff, int32_t size,
					  uart_unsol_rx_cb_t cb, void *param)
{

	__ASSERT(buff != NULL, "");
	__ASSERT(cb != NULL, "");
	__ASSERT(size != 0, "");

	if (k_sem_take(GET_RX_SEM(dev), K_NO_WAIT)) {
		LOG_ERR("Failed to acquire RX semaphore");
		return -EBUSY;
	}
	struct uart_sedi_drv_data *drv_data = dev->data;

	drv_data->unsol_rx_usr_cb = cb;
	drv_data->unsol_rx_usr_cb_param = param;
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	unsol_rx[instance].buffer = buff;
	unsol_rx[instance].size  = size;
	unsol_rx[instance].cb_data  = (void *)dev;
	unsol_rx[instance].unsol_rx_callback  = unsol_rx_cb;
	uart_busy_set(dev);
	if (sedi_uart_enable_unsol_rx(instance, &unsol_rx[instance])) {
		uart_busy_clear(dev);
		k_sem_give(GET_RX_SEM(dev));
		LOG_ERR("Failed to enable unsol RX");
		return -EIO;
	}
	return 0;
}

static int  uart_sedi_get_unsol_data(const struct device *dev, uint8_t *buff,
				     int32_t len)
{

	__ASSERT(buff != NULL, "");
	int ret;
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	ret = sedi_uart_get_unsol_data(instance, buff, len);
	if (ret != SEDI_DRIVER_OK) {
		LOG_ERR("Failed to get unsol data. ret:%d", ret);
		return -EIO;
	}
	return 0;
}

static int uart_sedi_disable_unsol_receive(const struct device *dev)
{

	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	if (sedi_uart_disable_unsol_rx(instance) != SEDI_DRIVER_OK) {
		LOG_ERR("Failed to disable unsol RX");
		return -EIO;
	}
	uart_busy_clear(dev);
	k_sem_give(GET_RX_SEM(dev));
	return 0;
}

static int uart_sedi_get_unsol_data_len(const struct device *dev, int *p_len)
{

	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	if (sedi_uart_get_unsol_data_len(instance, p_len) != SEDI_DRIVER_OK) {
		LOG_ERR("Failed to get unsol data length");
		return -EIO;
	}
	return 0;
}

static int uart_sedi_write_vec_async(const struct device *dev,
				     struct uart_io_vec *vec,
				     uint32_t count, uart_xfer_cb_t xfer_cb)
{
	__ASSERT(vec != NULL, "Write vec NULL!");
	__ASSERT(count != 0, "Write vec count zero!");
	int ret;

	if (k_sem_take(GET_TX_SEM(dev), K_NO_WAIT)) {
		return -EBUSY;
	}

	struct uart_sedi_drv_data *drv_data = dev->data;
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	drv_data->active_tx_xfer_cb = xfer_cb;
	vec_xfer_tx[instance].vec = (sedi_uart_io_vec_t *)vec;
	vec_xfer_tx[instance].count = count;
	vec_xfer_tx[instance].callback = uart_irq_xfer_tx_common_cb;
	vec_xfer_tx[instance].cb_data = (void *)dev;

	uart_busy_set(dev);
	ret = sedi_uart_write_vec_async(instance, &vec_xfer_tx[instance]);
	if (ret != SEDI_DRIVER_OK) {
		ret = get_xfer_error(ret);
		uart_busy_clear(dev);
		k_sem_give(GET_TX_SEM(dev));
		return ret;
	}

	return 0;
}

static int  uart_sedi_read_vec_async(const struct device *dev,
				     struct uart_io_vec *vec,
				     uint32_t count, uart_xfer_cb_t xfer_cb)
{
	__ASSERT(vec != NULL, "Write vec NULL!");
	__ASSERT(count != 0, "Write vec count zero!");

	int ret;

	if (k_sem_take(GET_RX_SEM(dev), K_NO_WAIT)) {
		LOG_ERR("Failed to acquire RX semaphore");
		return -EBUSY;
	}

	struct uart_sedi_drv_data *drv_data = dev->data;
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	drv_data->active_rx_xfer_cb = xfer_cb;
	vec_xfer_rx[instance].vec = (sedi_uart_io_vec_t *)vec;
	vec_xfer_rx[instance].count = count;
	vec_xfer_rx[instance].callback = uart_irq_xfer_rx_common_cb;
	vec_xfer_rx[instance].cb_data = (void *)dev;
	uart_busy_set(dev);
	ret = sedi_uart_read_vec_async(instance, &vec_xfer_rx[instance]);
	if (ret != SEDI_DRIVER_OK) {
		uart_busy_clear(dev);
		k_sem_give(GET_RX_SEM(dev));
		ret = get_xfer_error(ret);
		LOG_ERR("Read vector async failed. ret:%d", ret);
		return ret;
	}

	return 0;
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_LINE_CTRL
static int uart_sedi_line_ctrl_set(const struct device *dev,
				   uint32_t ctrl,
				   uint32_t val)
{
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);
	sedi_uart_config_t cfg = { 0 };
	uint32_t mask;
	int ret;

	k_mutex_lock(GET_MUTEX(dev), K_FOREVER);
	switch (ctrl) {
	case UART_LINE_CTRL_BAUD_RATE:
		sedi_uart_get_config(instance, &cfg);
		cfg.baud_rate = val;
		ret = sedi_uart_set_config(instance, &cfg);
		break;

	case UART_LINE_CTRL_BREAK_CONDN:
		if (val) {
			ret = sedi_uart_set_break_con(instance);
		} else {
			ret = sedi_uart_clr_break_con(instance);
		}
		break;

	case UART_LINE_CTRL_LOOPBACK:
		if (val) {
			ret = sedi_uart_set_loopback_mode(instance);
		} else {
			ret = sedi_uart_clr_loopback_mode(instance);
		}
		break;

	case UART_LINE_CTRL_AFCE:
		if (val) {
			ret = sedi_uart_auto_fc_enable(instance);
		} else {
			ret = sedi_uart_auto_fc_disable(instance);
		}
		break;

	case UART_LINE_CTRL_LINE_STATUS_REPORT_MASK:
		mask = 0;
		if (val  & UART_ERROR_OVERRUN) {
			mask |= SEDI_UART_RX_OE;
		}

		if (val & UART_ERROR_PARITY) {
			mask |= SEDI_UART_RX_PE;
		}

		if (val & UART_ERROR_FRAMING) {
			mask |= SEDI_UART_RX_FE;
		}

		if (val & UART_BREAK) {
			mask |= SEDI_UART_RX_BI;
		}
		ret = sedi_set_ln_status_report_mask(instance, mask);
		break;

	case UART_LINE_CTRL_RTS:
		if (val) {
			ret = sedi_uart_assert_rts(instance);
		} else {
			ret = sedi_uart_de_assert_rts(instance);
		}
		break;

#ifdef CONFIG_UART_RS_485
	case UART_LINE_CTRL_RS_485:
		if (val) {
			ret = sedi_uart_rs485_enable(instance);
		} else {
			ret = sedi_uart_rs485_disable(instance);
		}
		break;
#endif

#ifdef CONFIG_UART_9_BIT
	case UART_LINE_CTRL_9_BIT:
		if (val) {
			ret = sedi_uart_9bit_enable(instance);
		} else {
			ret = sedi_uart_9bit_disable(instance);
		}
		break;
#endif
	default:
		LOG_ERR("Invalid UART device");
		ret = -ENODEV;
	}
	k_mutex_unlock(GET_MUTEX(dev));
	ret = get_xfer_error(ret);
	if (ret != SEDI_DRIVER_OK) {
		LOG_ERR("Set line control failed. ret:%d", ret);
	}
	return ret;
}

static int uart_sedi_line_ctrl_get(const struct device *dev,
				   uint32_t ctrl,
				   uint32_t *val)
{
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);
	sedi_uart_config_t cfg = { 0 };
	uint32_t mask;
	int ret;

	k_mutex_lock(GET_MUTEX(dev), K_FOREVER);
	switch (ctrl) {
	case UART_LINE_CTRL_BAUD_RATE:
		ret = sedi_uart_get_config(instance, &cfg);
		*val = cfg.baud_rate;
		break;

	case UART_LINE_CTRL_LOOPBACK:
		ret = sedi_uart_get_loopback_mode(instance, (uint32_t *)val);
		break;

	case UART_LINE_CTRL_AFCE:
		ret = sedi_uart_get_config(instance, &cfg);
		*val = cfg.hw_fc;
		break;

	case UART_LINE_CTRL_LINE_STATUS_REPORT_MASK:
		mask = 0;
		*val = 0;
		ret = sedi_get_ln_status_report_mask(instance,
						     (uint32_t *)&mask);
		*val |= ((mask & SEDI_UART_RX_OE) ? UART_ERROR_OVERRUN : 0);
		*val |= ((mask & SEDI_UART_RX_PE) ? UART_ERROR_PARITY : 0);
		*val |= ((mask & SEDI_UART_RX_FE) ? UART_ERROR_FRAMING : 0);
		*val |= ((mask & SEDI_UART_RX_BI) ? UART_BREAK : 0);
		break;

	case UART_LINE_CTRL_RTS:
		ret = sedi_uart_read_rts(instance, (uint32_t *)val);
		break;

	case UART_LINE_CTRL_CTS:
		ret = sedi_uart_read_cts(instance, (uint32_t *)val);
		break;

	default:
		LOG_ERR("Invalid line control data");
		ret = -ENODEV;
	}
	k_mutex_unlock(GET_MUTEX(dev));
	ret = get_xfer_error(ret);
	if (ret != SEDI_DRIVER_OK) {
		LOG_ERR("Get line control failed. ret:%d", ret);
	}
	return ret;
}

#endif /* CONFIG_UART_LINE_CTRL */

#ifdef CONFIG_UART_DRV_CMD
static int uart_sedi_drv_cmd(const struct device *dev, uint32_t cmd, uint32_t p)
{
#if defined(CONFIG_UART_9_BIT) || defined(CONFIG_UART_RS_485)
	int ret;
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);
#endif

#ifdef CONFIG_UART_9_BIT
	if (cmd == UART_DRIVER_CMD_SEND_ADDR) {
		if (p > UART_NODE_ID_MASK) {
			return -EINVAL;
		}
		ret = sedi_uart_9bit_send_address(instance,
						  (uint8_t)(p &
							    UART_NODE_ID_MASK));
		ret = get_xfer_error(ret);
		if (ret != SEDI_DRIVER_OK) {
			LOG_ERR("Failed to send address. ret:%d", ret);
		}
		return ret;
	}
#endif

#ifdef CONFIG_UART_RS_485

	if (cmd == UART_DRIVER_CMD_SET_RX_ONLY_MODE) {
		ret = sedi_uart_set_rx_only_mode(instance, p);
		ret = get_xfer_error(ret);
		if (ret != SEDI_DRIVER_OK) {
			LOG_ERR("Failed to set RX only mode. ret:%d", ret);
		}
		return ret;
	}

	if (cmd == UART_DRIVER_CMD_SET_TX_ONLY_MODE) {
		ret = sedi_uart_set_tx_only_mode(instance, p);
		ret = get_xfer_error(ret);
		if (ret != SEDI_DRIVER_OK) {
			LOG_ERR("Failed to set TX only mode. ret:%d", ret);
		}
		return ret;
	}
#endif
	LOG_ERR("Invalid UART device");
	return -ENODEV;
}
#endif /* CONFIG_UART_DRV_CMD */

#ifdef CONFIG_UART_RS_485
static int uart_sedi_rs_485_config_set(const struct device *dev,
				       struct uart_rs_485_config *config)
{
	int ret;
	sedi_uart_rs485_config_t bsp_rs_485_config = { 0 };
	sedi_uart_t instance = GET_CONTROLLER_INSTANCE(dev);

	bsp_rs_485_config.de_assertion_time = config->de_assertion_time_ns;
	bsp_rs_485_config.de_deassertion_time = config->de_deassertion_time_ns;
	if (config->transfer_mode == UART_RS485_XFER_MODE_FULL_DUPLEX) {
		bsp_rs_485_config.transfer_mode =
			SEDI_UART_RS485_XFER_MODE_FULL_DUPLEX;
	} else {
		bsp_rs_485_config.transfer_mode =
			SEDI_UART_RS485_XFER_MODE_HALF_DUPLEX;
		bsp_rs_485_config.de_re_tat = config->de_re_tat_ns;
		bsp_rs_485_config.re_de_tat = config->re_de_tat_ns;
	}

	if (config->de_polarity == UART_RS485_POL_ACTIVE_LOW) {
		bsp_rs_485_config.de_polarity = SEDI_UART_RS485_POL_ACTIVE_LOW;
	} else {
		bsp_rs_485_config.de_polarity =
			SEDI_UART_RS485_POL_ACTIVE_HIGH;
	}

	if (config->re_polarity == UART_RS485_POL_ACTIVE_LOW) {
		bsp_rs_485_config.re_polarity =
			SEDI_UART_RS485_POL_ACTIVE_LOW;
	} else {
		bsp_rs_485_config.re_polarity =
			SEDI_UART_RS485_POL_ACTIVE_HIGH;
	}
	bsp_rs_485_config.de_en = true;
	bsp_rs_485_config.re_en = true;

	k_mutex_lock(GET_MUTEX(dev), K_FOREVER);
	ret = sedi_uart_rs485_set_config(instance, &bsp_rs_485_config);
	k_mutex_unlock(GET_MUTEX(dev));

	ret = get_xfer_error(ret);
	if (ret != SEDI_DRIVER_OK) {
		LOG_ERR("Device configuration error, ret:%d", ret);
	}
	return ret;
}
#endif

static const struct uart_driver_api api = {
#ifdef CONFIG_UART_ASYNC_API
	.callback_set = uart_sedi_callback_set,
	.tx = uart_sedi_tx,
	.tx_abort = uart_sedi_tx_abort,
	.rx_enable = uart_sedi_rx_enable,
	.rx_buf_rsp = uart_sedi_rx_buf_rsp,
	.rx_disable = uart_sedi_rx_disable,
#endif
	.poll_in = uart_sedi_poll_in,
	.poll_out = uart_sedi_poll_out,
	.err_check = uart_sedi_err_check,
#ifdef CONFIG_BOARD_EHL_PSE_CRB
	.read_buffer_polled = uart_sedi_read_buffer_polled,
	.write_buffer_polled = uart_sedi_write_buffer_polled,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_sedi_fifo_fill,
	.fifo_read = uart_sedi_fifo_read,
	.irq_tx_enable = uart_sedi_irq_tx_enable,
	.irq_tx_disable = uart_sedi_irq_tx_disable,
	.irq_tx_ready = uart_sedi_irq_tx_ready,
	.irq_tx_complete = uart_sedi_irq_tx_complete,
	.irq_rx_enable = uart_sedi_irq_rx_enable,
	.irq_rx_disable = uart_sedi_irq_rx_disable,
	.irq_rx_ready = uart_sedi_irq_rx_ready,
	.irq_err_enable = uart_sedi_irq_err_enable,
	.irq_err_disable = uart_sedi_irq_err_disable,
	.irq_is_pending = uart_sedi_irq_is_pending,
	.irq_update = uart_sedi_irq_update,
	.irq_callback_set = uart_sedi_irq_callback_set,
#ifdef CONFIG_BOARD_EHL_PSE_CRB
	.write_buffer_async = uart_sedi_write_buffer_async,
	.read_buffer_async = uart_sedi_read_buffer_async,
	.read_buffer_sync = uart_sedi_read_buffer_sync,
	.enable_unsol_receive = uart_sedi_enable_unsol_receive,
	.disable_unsol_receive = uart_sedi_disable_unsol_receive,
	.get_unsol_data = uart_sedi_get_unsol_data,
	.get_unsol_data_len = uart_sedi_get_unsol_data_len,
	.write_vec_async = uart_sedi_write_vec_async,
	.read_vec_async = uart_sedi_read_vec_async,
#endif  /* CONFIG_BOARD_EHL_PSE_CRB */
#endif  /* CONFIG_UART_INTERRUPT_DRIVEN */
#ifdef CONFIG_UART_LINE_CTRL
	.line_ctrl_set = uart_sedi_line_ctrl_set,
	.line_ctrl_get = uart_sedi_line_ctrl_get,
#endif  /* CONFIG_UART_LINE_CTRL */

#ifdef CONFIG_UART_DRV_CMD
	.drv_cmd = uart_sedi_drv_cmd,
#endif  /* CONFIG_UART_DRV_CMD */

#ifdef CONFIG_UART_RS_485
	.rs_485_config_set = uart_sedi_rs_485_config_set
#endif

};

void uart_sedi_clk_change_cb(uint32_t core_ferq, uint32_t hbw_freq,
			     uint32_t is_before, void *ctx)
{
	const struct uart_sedi_config_info *config =
		(const struct uart_sedi_config_info *) (ctx);

	if (!is_before) {
		sedi_uart_set_baud_rate(config->instance, config->baud_rate, hbw_freq);
	}
}

static int uart_sedi_init(const struct device *dev)
{

	const struct uart_sedi_config_info *config = dev->config;
	sedi_uart_config_t cfg = { 0 };
	struct uart_sedi_drv_data *drv_data = dev->data;

	if (DEV_PSE_OWNED !=
	    sedi_get_dev_ownership(PSE_DEV_UART0 + config->instance)) {
		return -ENODEV;
	}

	cfg.line_control = config->line_ctrl;
	cfg.baud_rate = config->baud_rate;
	cfg.hw_fc = config->hw_fc;
	cfg.clk_speed_hz = sedi_pm_get_hbw_clock();

	/* Setting to full power and enabling clk. */
	sedi_uart_set_power(config->instance, SEDI_POWER_FULL);

	sedi_uart_set_config(config->instance, &cfg);

	drv_data->pm_rst.func.clk_change_cb = uart_sedi_clk_change_cb;
	drv_data->pm_rst.ctx = (void *) config;
	drv_data->pm_rst.type = CALLBACK_TYPE_CLOCK_CHANGE;
	drv_data->pm_rst.pri = CALLBACK_PRI_NORMAL;
	sedi_pm_register_callback(&drv_data->pm_rst);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

	config->uart_irq_config_func(dev);
#endif  /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_9_BIT
	sedi_uart_9bit_config_t cfg_9_bit;

	cfg_9_bit.receive_address = config->node_id;
	cfg_9_bit.addr_ctrl = SEDI_UART_9BIT_HW_ADDR_CTRL;
	sedi_uart_9bit_set_config(config->instance, &cfg_9_bit);
#endif

	return 0;
}

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart0), okay)
UART_SEDI_DEVICE_INIT(0)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart1), okay)
UART_SEDI_DEVICE_INIT(1)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart2), okay)
UART_SEDI_DEVICE_INIT(2)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart3), okay)
UART_SEDI_DEVICE_INIT(3)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart4), okay)
UART_SEDI_DEVICE_INIT(4)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart5), okay)
UART_SEDI_DEVICE_INIT(5)
#endif
