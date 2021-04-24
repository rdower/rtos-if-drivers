# Copyright (c) 2021 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

if (CONFIG_HAS_INTEL)
  zephyr_library()

  set(INTEL_DIR ${ZEPHYR_CURRENT_MODULE_DIR}/)
  set(INC_DIR ${INTEL_DIR}/)
  set(SRC_DIR ${INTEL_DIR}/drivers/)

  zephyr_include_directories(${INC_DIR}/drivers)
  zephyr_include_directories(${INC_DIR}/include)
  zephyr_include_directories(${INC_DIR}/include/drivers)
  zephyr_include_directories(${INC_DIR}/soc/ehl/include)

  zephyr_library_sources_ifdef(CONFIG_INTEL_DMA    ${SRC_DIR}/dma/dma_ann_1p0.c)
  zephyr_library_sources_ifdef(CONFIG_INTEL_UART    ${SRC_DIR}/uart/dw_uart.c)
  zephyr_library_sources_ifdef(CONFIG_INTEL_TGPIO    ${SRC_DIR}/tgpio/intel_tgpio.c)
endif()
