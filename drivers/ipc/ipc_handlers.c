/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ipc.h>
#include <string.h>
#include <syscall_handler.h>

#define MAX_IPC_MSG_LEN 128

Z_SYSCALL_HANDLER(ipc_read_drbl, dev, drbl) {
	Z_OOPS(Z_SYSCALL_DRIVER_IPC(dev, ipc_read_drbl));
	Z_OOPS(Z_SYSCALL_MEMORY_WRITE(drbl, sizeof(uint32_t)));
	return z_impl_ipc_read_drbl((struct device *)dev, (uint32_t *)drbl);
}

Z_SYSCALL_HANDLER(ipc_read_msg, dev, drbl, msg, msg_size) {
	Z_OOPS(Z_SYSCALL_DRIVER_IPC(dev, ipc_read_msg));
	if (drbl) {
		Z_OOPS(Z_SYSCALL_MEMORY_WRITE(drbl, sizeof(uint32_t)));
	}
	if (msg) {
		Z_OOPS(Z_SYSCALL_MEMORY_WRITE(msg, msg_size));
	}
	return z_impl_ipc_read_msg((struct device *)dev,
				   (uint32_t *)drbl, (uint8_t *)msg, msg_size);
}

Z_SYSCALL_HANDLER(ipc_send_ack, dev, ack_drbl, ack_msg, ack_msg_size) {
	Z_OOPS(Z_SYSCALL_DRIVER_IPC(dev, ipc_send_ack));
	if (ack_msg) {
		Z_OOPS(Z_SYSCALL_MEMORY_READ(ack_msg, ack_msg_size));
	}
	return z_impl_ipc_send_ack((struct device *)dev,
				   ack_drbl, (uint8_t *)ack_msg, ack_msg_size);
}

Z_SYSCALL_HANDLER(ipc_write_msg, dev, drbl, msg, msg_size, ack_drbl,
		  more_args_ptr) {
	volatile struct _syscall_7_args *margs =
		(struct _syscall_7_args *)more_args_ptr;

	uint8_t *ack_msg = (uint8_t *)margs->arg6;
	uint32_t ack_msg_size = (uint32_t)margs->arg7;

	if (msg_size > MAX_IPC_MSG_LEN) {
		return -EINVAL;
	}
	uint8_t copy[msg_size];

	memcpy((void *)copy, (const void *)msg, msg_size);

	if (msg) {
		Z_OOPS(Z_SYSCALL_MEMORY_READ(msg, msg_size));
	}
	if (ack_drbl) {
		Z_OOPS(Z_SYSCALL_MEMORY_READ(ack_drbl, sizeof(uint32_t)));
	}
	if (ack_msg) {
		Z_OOPS(Z_SYSCALL_MEMORY_WRITE(ack_msg, ack_msg_size));
	}
	Z_OOPS(Z_SYSCALL_DRIVER_IPC(dev, ipc_write_msg));
	return z_impl_ipc_write_msg((struct device *)dev,
				    drbl, (uint8_t *)copy, msg_size,
				    (uint32_t *)ack_drbl, ack_msg, ack_msg_size);
}

