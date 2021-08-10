# SPDX-License-Identifier: Apache-2.0

target_sources_ifdef(CONFIG_GPIO_SEDI drivers__gpio PRIVATE gpio_sedi.c)
zephyr_library_sources_ifdef(CONFIG_GPIO_TIMED_SEDI gpio_timed_sedi.c)

zephyr_library_sources_ifdef(CONFIG_USERSPACE   gpio_timed_handlers.c)
