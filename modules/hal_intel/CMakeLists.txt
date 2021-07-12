# Copyright (c) 2021 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

if (CONFIG_HAS_INTEL)
  zephyr_library()

  set(INTEL_DIR ${ZEPHYR_CURRENT_MODULE_DIR}/)
  set(INC_DIR ${INTEL_DIR}/)
  set(SRC_DIR ${INTEL_DIR}/drivers/)

  zephyr_include_directories(${INC_DIR}/include)
  zephyr_include_directories(${INC_DIR}/soc/ehl/include)

  zephyr_library_sources(${SRC_DIR}/common/hal_driver_common.c)
  zephyr_library_sources_ifdef(CONFIG_INTEL_DMA    ${SRC_DIR}/dma/dma_ann_1p0.c)
  zephyr_library_sources_ifdef(CONFIG_INTEL_UART    ${SRC_DIR}/uart/dw_uart.c)
  zephyr_library_sources_ifdef(CONFIG_INTEL_SPI    ${SRC_DIR}/spi/dw_spi.c)
  zephyr_library_sources_ifdef(CONFIG_INTEL_TGPIO    ${SRC_DIR}/tgpio/intel_tgpio.c)
endif()

if (CONFIG_HAS_SEDI)
  set(SEDI_DIR ${ZEPHYR_CURRENT_MODULE_DIR}/)
  set(sedi_lib_src_dir ${ZEPHYR_CURRENT_MODULE_DIR}/soc/arm/elkhart_lake_pse/sedi)

  zephyr_interface_library_named(INTEL_BSP_SEDI)
  zephyr_library()
  zephyr_library_link_libraries(INTEL_BSP_SEDI)
  target_link_libraries(zephyr_interface INTERFACE INTEL_BSP_SEDI)

  target_include_directories(INTEL_BSP_SEDI INTERFACE ${sedi_lib_src_dir}/include)
  target_include_directories(INTEL_BSP_SEDI INTERFACE ${sedi_lib_src_dir}/include/driver)

  zephyr_library_include_directories(${sedi_lib_src_dir}/drivers)
  zephyr_include_directories(${sedi_lib_src_dir}/soc/ehl_pse/include)

  zephyr_library_include_directories(${sedi_lib_src_dir}/include)
  zephyr_library_sources(${sedi_lib_src_dir}/drivers/common/sedi_common.c)
  zephyr_library_sources(${sedi_lib_src_dir}/drivers/pm/pm.c ${sedi_lib_src_dir}/drivers/pm/pm.S)

  zephyr_library_sources_ifdef(CONFIG_DMA_SEDI    ${sedi_lib_src_dir}/drivers/dma/dma_ann_1p0.c)
  zephyr_library_sources_ifdef(CONFIG_UART_SEDI   ${sedi_lib_src_dir}/drivers/usart/dw_uart.c)
  zephyr_library_sources_ifdef(CONFIG_QEP_SEDI   ${sedi_lib_src_dir}/drivers/qep/qep_sedi.c)
  zephyr_library_sources_ifdef(CONFIG_CAN_SEDI   ${sedi_lib_src_dir}/drivers/can/sedi_driver_can.c)
  zephyr_library_sources(${sedi_lib_src_dir}/drivers/ipc/ipc_bsp.c
    ${sedi_lib_src_dir}/drivers/ipc/fwst.c)
  zephyr_library_sources_ifdef(CONFIG_HPET_TIMER ${sedi_lib_src_dir}/drivers/hpet/sedi_hpet.c)

endif()
