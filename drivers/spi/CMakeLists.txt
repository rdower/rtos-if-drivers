# Copyright (c) 2021 Intel Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SPI_INTEL spi_intel.c)
