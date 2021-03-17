# Copyright (c) 2021 Intel Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_UART_INTEL uart_intel.c)

target_sources_ifdef(CONFIG_UART_SEDI drivers__serial PRIVATE uart_sedi.c)
