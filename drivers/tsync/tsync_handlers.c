/*
 * Copyright (c) 2022 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <syscall_handler.h>
#include <drivers/tsync.h>

static inline int z_vrfy_tsync_sync(const struct device *dev)
{
	return z_impl_tsync_sync((const struct device *)dev);
}

#include <syscalls/tsync_sync_mrsh.c>

static inline int z_vrfy_tsync_get_time(const struct device *dev, uint64_t *timestamp)
{
	return z_impl_tsync_get_time((struct device *)dev, (uint64_t *)timestamp);
}

#include <syscalls/tsync_get_time_mrsh.c>
