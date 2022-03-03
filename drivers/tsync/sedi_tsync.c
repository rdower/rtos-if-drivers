/*
 * Copyright (c) 2022 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <drivers/tsync.h>
#include <kernel.h>
#include <sedi.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(TSYNC, CONFIG_TSYNC_LOG_LEVEL);

#define LOCAL_SYNC_OPCODE (0x51)

static void tsync_sx_handler(sedi_pm_sx_event_t sx_event, void *ctx)
{
	PARAM_UNUSED(ctx);

	if (sx_event == PM_EVENT_HOST_SX_EXIT) {
		sedi_tsync_sync();
	}
}

#ifdef CONFIG_PM_DEVICE

static uint32_t tsync_resume_device_from_suspend(const struct device *dev)
{
	uint32_t status = 0;

	/* Make sure sideband already resumed */
	sedi_sideband_set_power(SEDI_SIDEBAND_0, SEDI_POWER_FULL);
	/* After sideband resume, resume tsync */
	status = sedi_sideband_register_client(SEDI_SIDEBAND_0, SB_DOWN_TSYNC,
					       LOCAL_SYNC_OPCODE, NULL, NULL);

	return status;
}

static int tsync_sedi_device_ctrl(const struct device *dev,
				enum pm_device_action action)
{
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		break;
	case PM_DEVICE_ACTION_RESUME:
		ret = tsync_resume_device_from_suspend(dev);
		break;
	case PM_DEVICE_ACTION_LOW_POWER:
		/* Always-on module, nothing to do */
		break;
	case PM_DEVICE_ACTION_FORCE_SUSPEND:
		break;
	default:
		ret = -ENOTSUP;
	}

	return ret;
}

#endif /* CONFIG_DEVICE_POWER_MANAGEMENT */

static int tsync_sedi_sync(const struct device *dev)
{
	return sedi_tsync_sync();
}

static int tsync_sedi_get_time(const struct device *dev, uint64_t *timestamp)
{
	return sedi_tsync_get_time(timestamp);
}

static const struct tsync_driver_api sedi_tsync_driver_api = {
	.sync = tsync_sedi_sync,
	.get_time = tsync_sedi_get_time,
};

static int tsync_sedi_init(const struct device *dev)
{
	sedi_sideband_register_client(SEDI_SIDEBAND_0, SB_DOWN_TSYNC,
				      LOCAL_SYNC_OPCODE, NULL, NULL);
	sedi_tsync_sync();
	/* Register boot prepare callback */
	sedi_pm_register_sx_notification(tsync_sx_handler, NULL);

	return 0;
}

DEVICE_DEFINE(tsync, "TSYNC", &tsync_sedi_init, tsync_sedi_device_ctrl, NULL, NULL,
	      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
	      &sedi_tsync_driver_api);
