# SPDX-License-Identifier: Apache-2.0

target_sources_ifdef(CONFIG_GPIO_SEDI drivers__gpio PRIVATE gpio_sedi.c)
