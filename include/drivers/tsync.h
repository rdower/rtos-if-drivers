/*
 * Copyright (c) 2022 Intel Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_TSYNC_H_
#define ZEPHYR_INCLUDE_TSYNC_H_

/**
 * @file
 * @brief Public TSYNC driver APIs
 */

#include <zephyr/types.h>
#include <device.h>

/**
 * @typedef tsync_api_sync()
 * @brief Callback API for time-synchronization sync.
 *
 * @see tsync_sync() for argument descriptions.
 */
typedef int (*tsync_api_sync)(const struct device *dev);

/**
 * @typedef tsync_api_get_time()
 * @brief Callback API for get time-synchronization timestamp.
 *
 * @see tsync_get_time() for argument descriptions.
 */
typedef int (*tsync_api_get_time)(const struct device *dev, uint64_t *timestamp);


/**
 * @brief TSYNC driver API
 *
 * This is the mandatory API any TSYNC driver needs to expose.
 */
struct tsync_driver_api {
	tsync_api_sync sync;
	tsync_api_get_time get_time;
};

/**
 * @brief time-synchronization sync
 *
 * This routine finished a time-synchronization sync process
 *
 * @param dev TSYNC device
 * @return 0 on success, negative on error
 */
__syscall int tsync_sync(const struct device *dev);

static inline int z_impl_tsync_sync(const struct device *dev)
{
	const struct tsync_driver_api *api = dev->api;

	return api->sync(dev);
}

/**
 * @brief Get time-synchronization timestamp
 *
 * This routine get the timestamp from time-synchronization. This routine shall
 * called after tsync_sync function to keep the time is accurate.
 *
 * @param dev TSYNC device
 * @param timestamp Pointer to timestamp get from time-synchronization.
 * @return 0 on success, negative on error
 */
__syscall int tsync_get_time(const struct device *dev, uint64_t *timestamp);

static inline int z_impl_tsync_get_time(const struct device *dev, uint64_t *timestamp)
{
	const struct tsync_driver_api *api = dev->api;

	return api->get_time(dev, timestamp);
}

#include <syscalls/tsync.h>

#endif  /* ZEPHYR_INCLUDE_TSYNC_H_ */
