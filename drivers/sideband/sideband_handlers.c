/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/sideband.h>
#include <syscall_handler.h>
#include <string.h>

static inline int z_vrfy_sideband_send(const struct device *dev, uint32_t port,
				       uint32_t action, uint64_t addr,
				       uint32_t data)
{
	Z_OOPS(Z_SYSCALL_DRIVER_SIDEBAND(dev, send));
	return z_impl_sideband_send((const struct device *)dev, port, action,
				    (uint64_t)addr, data);
}
#include <syscalls/sideband_send_mrsh.c>

static inline int z_vrfy_sideband_wait_ack(const struct device *dev,
					   uint32_t port, uint32_t action,
					   uint32_t *data)
{
	Z_OOPS(Z_SYSCALL_DRIVER_SIDEBAND(dev, wait_ack));
	return z_impl_sideband_wait_ack((const struct device *)dev, port, action,
					(uint32_t *)data);
}
#include <syscalls/sideband_wait_ack_mrsh.c>

static inline int z_vrfy_sideband_receive(const struct device *dev,
					  uint32_t port, uint64_t *addr,
					  uint64_t *data)
{
	Z_OOPS(Z_SYSCALL_DRIVER_SIDEBAND(dev, receive));
	return z_impl_sideband_receive((const struct device *)dev, port,
				       (uint64_t *)addr, (uint64_t *)data);
}
#include <syscalls/sideband_receive_mrsh.c>

static inline int z_vrfy_sideband_send_ack(const struct device *dev,
					   uint32_t port, uint32_t action,
					   uint32_t data)
{
	Z_OOPS(Z_SYSCALL_DRIVER_SIDEBAND(dev, send_ack));
	return z_impl_sideband_send_ack((const struct device *)dev, port, action,
					data);
}
#include <syscalls/sideband_send_ack_mrsh.c>

static inline int z_vrfy_sideband_write_raw(const struct device *dev,
					    struct sb_raw_message *message)
{
	Z_OOPS(Z_SYSCALL_DRIVER_SIDEBAND(dev, write_raw));
	return z_impl_sideband_write_raw((const struct device *)dev,
					 (struct sb_raw_message *)message);
}
#include <syscalls/sideband_write_raw_mrsh.c>

static inline int z_vrfy_sideband_read_raw(const struct device *dev,
					   struct sb_raw_message *message)
{
	Z_OOPS(Z_SYSCALL_DRIVER_SIDEBAND(dev, read_raw));
	return z_impl_sideband_read_raw((const struct device *)dev,
					(struct sb_raw_message *)message);
}
#include <syscalls/sideband_read_raw_mrsh.c>

static inline int z_vrfy_sideband_register_client(const struct device *dev,
						  uint32_t client,
						  uint32_t opcode)
{
	Z_OOPS(Z_SYSCALL_DRIVER_SIDEBAND(dev, register_client));
	return z_impl_sideband_register_client((const struct device *)dev,
					       client, opcode);
}
#include <syscalls/sideband_register_client_mrsh.c>

static inline int z_vrfy_sideband_unregister_client(const struct device *dev,
						    uint32_t client)
{
	Z_OOPS(Z_SYSCALL_DRIVER_SIDEBAND(dev, unregister_client));
	return z_impl_sideband_unregister_client((const struct device *)dev,
						 client);
}
#include <syscalls/sideband_unregister_client_mrsh.c>
