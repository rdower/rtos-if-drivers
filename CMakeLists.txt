# Copyright (c) 2020 Intel Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(include)
zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/../zephyr/drivers/spi)
add_subdirectory(drivers)
