# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_USERSPACE   sideband_handlers.c)
zephyr_sources_ifdef(CONFIG_SIDEBAND sedi_sideband.c)

