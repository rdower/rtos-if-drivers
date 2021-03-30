# Copyright (c) 2021 Intel Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_PSE_HPET_TIMER pse_hpet.c)
