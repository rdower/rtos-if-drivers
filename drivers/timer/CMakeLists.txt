# Copyright (c) 2020 Intel Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_ISH_HPET_TIMER ish_hpet.c)
