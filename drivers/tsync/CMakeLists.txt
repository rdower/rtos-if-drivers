# SPDX-License-Identifier: Apache-2.0
zephyr_sources_ifdef(CONFIG_TSYNC sedi_tsync.c)
zephyr_sources_ifdef(CONFIG_USERSPACE   tsync_handlers.c)
