/*
 * Copyright (c) 2020-2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT intel_sedi_ipc
#include <kernel.h>
#include <init.h>
#include <errno.h>
#include <device.h>
#include <sys/sys_io.h>
#include <drivers/ipc.h>
#include "ipc_sedi.h"

#include <devicetree.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(ipc_sedi, CONFIG_IPC_LOG_LEVEL);

#ifdef CONFIG_SHARED_IRQ
#include <shared_irq.h>
#endif
#ifdef CONFIG_IOAPIC
#include <drivers/ioapic.h>
#endif


void sedi_ipc_isr(IN sedi_ipc_t ipc_device);

K_SEM_DEFINE(csr_sem, 0, 1);

static void set_ipc_dev_busy(const struct device *dev, bool is_write)
{
	struct ipc_sedi_context *ipc = dev->data;
	unsigned int key = irq_lock();

	atomic_set_bit(&ipc->status,
		       is_write ? IPC_WRITE_BUSY_BIT : IPC_READ_BUSY_BIT);
	device_busy_set(dev);
	irq_unlock(key);
}

static void clear_ipc_dev_busy(const struct device *dev, bool is_write)
{
	struct ipc_sedi_context *ipc = dev->data;
	unsigned int key = irq_lock();

	atomic_clear_bit(&ipc->status,
			 is_write ? IPC_WRITE_BUSY_BIT : IPC_READ_BUSY_BIT);
	if (!atomic_test_bit(&ipc->status,
			is_write ? IPC_READ_BUSY_BIT : IPC_WRITE_BUSY_BIT)) {
		device_busy_clear(dev);
	}
	irq_unlock(key);
}

void csr_state_machine(const struct device *dev, uint32_t csr)
{
	struct ipc_sedi_context *ipc = dev->data;
	const struct ipc_sedi_config_t *info = dev->config;

	sedi_ipc_t device = info->ipc_device;

	LOG_DBG(" %08x", csr);
	if (csr & IPC_CSR_ACKED_VALID) {
		k_sem_give(&csr_sem);
	}

	if ((csr & IPC_CSR_RESET_EXIT) || (csr & IPC_CSR_QUERY)) {
		atomic_set_bit(&ipc->status, IPC_PEER_READY_BIT);
		sedi_ipc_write_csr(device, IPC_CSR_RESET_EXIT);
		return;
	}

	if (csr & IPC_CSR_RESET_ENTRY) {
		atomic_clear_bit(&ipc->status, IPC_PEER_READY_BIT);
		sedi_ipc_write_csr(device, IPC_CSR_RESET_ENTRY);
	}
}

static void ipc_event_dispose(IN sedi_ipc_t device, IN uint32_t event,
			      INOUT void *params)
{
	const struct device *dev = (const struct device *)params;
	struct ipc_sedi_context *ipc = dev->data;
	const struct ipc_sedi_config_t *info = dev->config;
	uint32_t csr = 0;

	LOG_DBG("dev: %u, event: %u", device, event);
	switch (event) {
	case SEDI_IPC_EVENT_MSG_IN:
		if (ipc->rx_msg_notify_cb != NULL) {
			set_ipc_dev_busy(dev, false);
			ipc->rx_msg_notify_cb(dev, NULL);
		} else {
			LOG_WRN("no handler for ipc new msg");
		}
		break;
	case SEDI_IPC_EVENT_MSG_PEER_ACKED:
		if (atomic_test_bit(&ipc->status, IPC_WRITE_IN_PROC_BIT)) {
			k_sem_give(&ipc->device_write_msg_sem);
		}
		break;
	case SEDI_IPC_EVENT_CSR_ACK:
		if (info->need_sync) {
			sedi_ipc_read_csr(device, &csr);
			csr_state_machine(dev, csr);
		}
		break;
	default:
		return;
	}
}

#define IPC_CONNECT_IRQ(inst, sedi_inst, sense)				    \
	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(inst)),			    \
			DT_IRQ(DT_NODELABEL(inst), priority),		    \
			sedi_ipc_isr,					    \
			(void *)sedi_inst,				    \
			sense);
static int ipc_init(const struct device *dev)
{
	__ASSERT((dev != NULL), "bad params\n");
	/* allocate resource and context*/
	const struct ipc_sedi_config_t *info = dev->config;
	sedi_ipc_t device = info->ipc_device;
	struct ipc_sedi_context *ipc = dev->data;

	switch (device) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(ipchost), okay)
	case SEDI_IPC_HOST:
		IPC_CONNECT_IRQ(ipchost, SEDI_IPC_HOST,
#if DT_IRQ_HAS_CELL(DT_NODELABEL(ipchost), sense)
			DT_IRQ(DT_NODELABEL(ipchost), sense)
#else
			0
#endif
		);
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(ipccsme), okay)
	case SEDI_IPC_CSME:
		IPC_CONNECT_IRQ(ipccsme, SEDI_IPC_CSME,
#if DT_IRQ_HAS_CELL(DT_NODELABEL(ipccsme), sense)
			DT_IRQ(DT_NODELABEL(ipccsme), sense)
#else
			0
#endif
			);
		break;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(ipcpmc), okay)
	case SEDI_IPC_PMC:
		IPC_CONNECT_IRQ(ipcpmc, SEDI_IPC_PMC,
#if DT_IRQ_HAS_CELL(DT_NODELABEL(ipcpmc), sense)
			DT_IRQ(DT_NODELABEL(ipcpmc), sense)
#else
			0
#endif
			);
		break;
#endif
	default:
		return -EIO;
	}

	k_sem_init(&ipc->device_write_msg_sem, 0, 1);
	k_mutex_init(&ipc->device_write_lock);
	ipc->status = 0;

#if defined(CONFIG_PM_DEVICE)
	ipc->power_status = PM_DEVICE_STATE_ACTIVE;
#endif

	sedi_ipc_init(device, ipc_event_dispose, (void *)dev);

	if (!info->need_sync) {
		atomic_set_bit(&ipc->status, IPC_PEER_READY_BIT);
	}
	LOG_DBG("IPC driver initialized on device: %p", dev);
	return 0;
}

static int write_msg(const struct device *dev,
		     uint32_t drbl,
		     uint8_t *msg,
		     uint32_t msg_size,
		     uint32_t *ack_drbl,
		     uint8_t *ack_msg,
		     uint32_t ack_msg_size)
{
	__ASSERT((dev != NULL), "bad params\n");
	const struct ipc_sedi_config_t *info = dev->config;
	struct ipc_sedi_context *ipc = dev->data;
	sedi_ipc_t device = info->ipc_device;
	int ret;

	/* check params, check status */
	if ((msg_size > IPC_DATA_LEN_MAX)
	    || ((msg_size > 0) && (msg == NULL))
	    || (ack_msg_size > IPC_DATA_LEN_MAX)
	    || ((ack_msg_size > 0) && (ack_msg == NULL))
	    || ((drbl & BIT(IPC_BUSY_BIT)) == 0)) {
		LOG_ERR("bad params when sending ipc msg on device: %p", dev);
		return -EINVAL;
	}

	k_mutex_lock(&ipc->device_write_lock, K_FOREVER);
	set_ipc_dev_busy(dev, true);

	if (!atomic_test_bit(&ipc->status, IPC_PEER_READY_BIT)) {
		LOG_WRN("peer is not ready");
		goto write_err;
	}

	if (info->need_sync) {
		sedi_ipc_write_csr(device, IPC_CSR_ASSERT_VALID);
		ret = k_sem_take(&csr_sem, K_MSEC(info->default_timeout));
		if (ret) {
			LOG_WRN("IPC wait csr timeout on device: %p", dev);
			sedi_ipc_write_csr(device, IPC_CSR_DEASSERT_VALID);
			goto write_err;
		}

	}

	/* write data regs */
	if (msg_size > 0) {
		ret = sedi_ipc_write_msg(device, msg, msg_size);
		if (ret != SEDI_DRIVER_OK) {
			LOG_ERR("IPC write data fail on device: %p", dev);
			if (info->need_sync) {
				sedi_ipc_write_csr(device,
						   IPC_CSR_DEASSERT_VALID);
			}
			goto write_err;
		}
	}

	atomic_set_bit(&ipc->status, IPC_WRITE_IN_PROC_BIT);
	/* write drbl regs to interrupt peer*/
	ret = sedi_ipc_write_dbl(device, drbl);

	if (info->need_sync) {
		sedi_ipc_write_csr(device, IPC_CSR_DEASSERT_VALID);
	}

	if (ret != SEDI_DRIVER_OK) {
		LOG_ERR("IPC write doorbell fail on device: %p", dev);
		atomic_clear_bit(&ipc->status, IPC_WRITE_IN_PROC_BIT);
		goto write_err;
	}

	/* wait for busy-bit-consumed interrupt */
	ret = k_sem_take(&ipc->device_write_msg_sem,
			 K_MSEC(info->default_timeout));
	if (ret) {
		LOG_WRN("IPC write timeout on device: %p", dev);
		atomic_clear_bit(&ipc->status, IPC_WRITE_IN_PROC_BIT);
		sedi_ipc_write_dbl(device, 0);
		goto write_err;
	}

	if (ack_msg_size > 0) {
		ret = sedi_ipc_read_ack_msg(device, ack_msg, ack_msg_size);
		if (ret) {
			LOG_ERR("IPC read ack failed on device: %p", dev);
			atomic_clear_bit(&ipc->status, IPC_WRITE_IN_PROC_BIT);
			goto write_err;
		}
	}

	if (ack_drbl) {
		ret = sedi_ipc_read_ack_drbl(device, ack_drbl);
		if (ret) {
			LOG_ERR("IPC read ack failed on device: %p", dev);
			atomic_clear_bit(&ipc->status, IPC_WRITE_IN_PROC_BIT);
			goto write_err;
		}
	}

	atomic_clear_bit(&ipc->status, IPC_WRITE_IN_PROC_BIT);
	clear_ipc_dev_busy(dev, true);

	k_mutex_unlock(&ipc->device_write_lock);
	LOG_DBG("IPC wrote a new message on device: %p, drbl=%08x", dev, drbl);

	return 0;

write_err:
	clear_ipc_dev_busy(dev, true);
	k_mutex_unlock(&ipc->device_write_lock);
	return -1;
}

static int set_rx_notify(const struct device *dev, ipc_new_msg_f cb)
{
	__ASSERT((dev != NULL), "bad params\n");

	const struct ipc_sedi_config_t *info = dev->config;
	struct ipc_sedi_context *ipc = dev->data;

	sedi_ipc_t device = info->ipc_device;

	if (cb == NULL) {
		LOG_ERR("bad params when add ipc callback on device: %p", dev);
		return -EINVAL;
	}

	if (ipc->rx_msg_notify_cb == NULL) {
		ipc->rx_msg_notify_cb = cb;
		if (info->need_sync) {
			sedi_ipc_write_csr(device, IPC_CSR_RESET_ENTRY);
		}
		irq_enable(info->irq_num);
		return 0;
	}

	LOG_ERR("ipc rx callback already exists on device: %p", dev);
	return -1;
}

static int read_drbl(const struct device *dev, uint32_t *drbl)
{
	__ASSERT((dev != NULL), "bad params\n");
	if (drbl == NULL) {
		LOG_ERR("bad params when read drbl on device: %p", dev);
		return -EINVAL;
	}
	int ret;
	const struct ipc_sedi_config_t *info = dev->config;
	sedi_ipc_t device = info->ipc_device;

	ret = sedi_ipc_read_dbl(device, drbl);
	if (ret != SEDI_DRIVER_OK) {
		LOG_ERR("IPC read doorbell fail on device: %p", dev);
		return -1;
	} else {
		return 0;
	}
}

static int read_msg(const struct device *dev, uint32_t *drbl,
		    uint8_t *msg, uint32_t msg_size)
{
	__ASSERT((dev != NULL), "bad params\n");
	int ret;
	const struct ipc_sedi_config_t *info = dev->config;
	sedi_ipc_t device = info->ipc_device;

	if (drbl) {
		ret = sedi_ipc_read_dbl(device, drbl);
		if (ret != SEDI_DRIVER_OK) {
			LOG_ERR("IPC read drbl fail on device: %p", dev);
			return -1;
		}
	}
	if (msg_size == 0) {
		return 0;
	}
	if (msg == NULL) {
		LOG_ERR("bad params when read data on device: %p", dev);
		return -EINVAL;
	}
	ret = sedi_ipc_read_msg(device, msg, msg_size);
	if (ret != SEDI_DRIVER_OK) {
		LOG_ERR("IPC read data failed on device: %p", dev);
		return -1;
	} else {
		return 0;
	}
}

static int send_ack(const struct device *dev,
		    uint32_t ack_drbl,
		    uint8_t *ack_msg,
		    uint32_t ack_msg_size)
{
	int ret;

	__ASSERT((dev != NULL), "bad params\n");
	if ((ack_drbl & BIT(IPC_BUSY_BIT))
	    || (ack_msg_size > IPC_DATA_LEN_MAX)
	    || ((ack_msg_size > 0) && (ack_msg == NULL))) {
		LOG_ERR("bad params when sending ack on device: %p", dev);
		return -1;
	}

	const struct ipc_sedi_config_t *info = dev->config;
	sedi_ipc_t device = info->ipc_device;

	if (ack_msg != NULL) {
		ret = sedi_ipc_send_ack_msg(device, ack_msg, ack_msg_size);
		if (ret != SEDI_DRIVER_OK) {
			LOG_ERR("IPC send ack msg fail on device: %p", dev);
			goto ack_end;
		}
	}

	ret = sedi_ipc_send_ack_drbl(device, ack_drbl);
	if (ret != SEDI_DRIVER_OK) {
		LOG_ERR("IPC send ack drl fail on device: %p", dev);
	}

ack_end:
	clear_ipc_dev_busy(dev, false);
	return ret;
}

#if defined(CONFIG_PM_DEVICE)
static int ipc_power_ctrl(const struct device *dev, uint32_t ctrl_command,
			  uint32_t *context, pm_device_cb cb, void *arg)
{
	int ret = 0;
	struct ipc_sedi_context *data = dev->data;

	if (ctrl_command == PM_DEVICE_STATE_SET) {
		data->power_status = *((uint32_t *)context);
	} else if (ctrl_command == PM_DEVICE_STATE_GET) {
		*context = data->power_status;
	}

	if (cb) {
		cb(dev, ret, context, arg);
	}

	return ret;

}
#endif

struct ipc_driver_api ipc_api = {
	.ipc_set_rx_notify = set_rx_notify,
	.ipc_read_drbl = read_drbl,
	.ipc_read_msg = read_msg,
	.ipc_send_ack = send_ack,
	.ipc_write_msg = write_msg
};

#if DT_NODE_HAS_STATUS(DT_NODELABEL(ipchost), okay)
static struct ipc_sedi_context ipc_data_host;
static struct ipc_sedi_config_t ipc_config_host = {
	.ipc_device = SEDI_IPC_HOST,
	.irq_num = DT_IRQN(DT_NODELABEL(ipchost)),
	.need_sync = false,
	.default_timeout = DT_PROP(DT_NODELABEL(ipchost), timeout_ms)
};
DEVICE_DEFINE(ipc_host, DT_LABEL(DT_NODELABEL(ipchost)), &ipc_init,
	      ipc_power_ctrl,
	      &ipc_data_host, &ipc_config_host,
	      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
	      &ipc_api);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(ipccsme), okay)
static struct ipc_sedi_context ipc_data_csme;
static struct ipc_sedi_config_t ipc_config_csme = {
	.ipc_device = SEDI_IPC_CSME,
	.irq_num = DT_IRQN(DT_NODELABEL(ipccsme)),
	.need_sync = true,
	.default_timeout = DT_PROP(DT_NODELABEL(ipccsme), timeout_ms)
};
DEVICE_DEFINE(ipc_csme, DT_LABEL(DT_NODELABEL(ipccsme)), &ipc_init,
	      ipc_power_ctrl,
	      &ipc_data_csme, &ipc_config_csme,
	      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
	      &ipc_api);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(ipcpmc), okay)
static struct ipc_sedi_context ipc_data_pmc;
static struct ipc_sedi_config_t ipc_config_pmc = {
	.ipc_device = SEDI_IPC_PMC,
	.irq_num = DT_IRQN(DT_NODELABEL(ipcpmc)),
	.need_sync = false,
	.default_timeout = DT_PROP(DT_NODELABEL(ipcpmc), timeout_ms)
};
DEVICE_DEFINE(ipc_pmc, DT_LABEL(DT_NODELABEL(ipcpmc)), &ipc_init,
	      ipc_power_ctrl,
	      &ipc_data_pmc, &ipc_config_pmc,
	      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
	      &ipc_api);
#endif

