/*
 * Copyright (c) 2020-2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __DRIVERS_IPC_SEDI_H
#define __DRIVERS_IPC_SEDI_H

#ifdef __cplusplus
extern "C" {
#endif
#include "sedi_driver_common.h"
#include "sedi_driver_ipc.h"
#include "sys/atomic.h"

#define IPC_BUSY_BIT            31
#define IPC_WRITE_TINEOUT_1S    1000
#define IPC_WRITE_TINEOUT_5S    5000
#define IPC_WRITE_IN_PROC_BIT   0
#define IPC_WRITE_BUSY_BIT      1
#define IPC_READ_BUSY_BIT       2
#define IPC_PEER_READY_BIT      3

#define VALID_DRBL(drbl) (drbl & BIT(IPC_BUSY_BIT))

typedef void (*ipc_config_func_t)(void);

struct ipc_sedi_config_t {
	sedi_ipc_t ipc_device;
	int32_t default_timeout;
	int32_t irq_num;
	bool need_sync;
};

struct ipc_sedi_context {
	ipc_new_msg_f rx_msg_notify_cb;
	struct k_sem device_write_msg_sem;
	struct k_mutex device_write_lock;
	atomic_t status;
	uint32_t power_status;
};

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_IPC_SEDI_H */
