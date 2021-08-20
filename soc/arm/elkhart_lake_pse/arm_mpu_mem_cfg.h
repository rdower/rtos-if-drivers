/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _ARM_MPU_MEM_CFG_H_
#define _ARM_MPU_MEM_CFG_H_

#include <soc.h>
#include <arch/arm/aarch32/cortex_m/mpu/arm_mpu.h>

#define MEM_SIZE_1K    (1024)
#define MEM_SIZE_2K    (MEM_SIZE_1K * 2)
#define MEM_SIZE_4K    (MEM_SIZE_1K * 4)
#define MEM_SIZE_8K    (MEM_SIZE_1K * 8)
#define MEM_SIZE_16K   (MEM_SIZE_1K * 16)
#define MEM_SIZE_32K   (MEM_SIZE_1K * 32)
#define MEM_SIZE_64K   (MEM_SIZE_1K * 64)
#define MEM_SIZE_128K  (MEM_SIZE_1K * 128)
#define MEM_SIZE_256K  (MEM_SIZE_1K * 256)
#define MEM_SIZE_512K  (MEM_SIZE_1K * 512)
#define MEM_SIZE_1M    (MEM_SIZE_1K * 1024)

#define MEM_SIZE_192K  (MEM_SIZE_1K * 192)
#define MEM_SIZE_320K  (MEM_SIZE_1K * 320)
#define MEM_SIZE_384K  (MEM_SIZE_1K * 384)

#if (CONFIG_ICCM_MEMORY_SIZE == 0)
#define PSE_ICCM_RASR_SRD (MPU_RASR_SRD_Msk)
#elif (CONFIG_ICCM_MEMORY_SIZE == MEM_SIZE_64K)
#define PSE_ICCM_RASR_SRD (MPU_RASR_SRD_Msk &	\
			     ~(SUB_REGION_0_DISABLED))
#elif (CONFIG_ICCM_MEMORY_SIZE == MEM_SIZE_128K)
#define PSE_ICCM_RASR_SRD (MPU_RASR_SRD_Msk &	       \
			     ~(SUB_REGION_0_DISABLED | \
			       SUB_REGION_1_DISABLED))
#elif (CONFIG_ICCM_MEMORY_SIZE == MEM_SIZE_192K)
#define PSE_ICCM_RASR_SRD (MPU_RASR_SRD_Msk &	       \
			     ~(SUB_REGION_0_DISABLED | \
			       SUB_REGION_1_DISABLED | \
			       SUB_REGION_2_DISABLED))
#elif (CONFIG_ICCM_MEMORY_SIZE == MEM_SIZE_256K)
#define PSE_ICCM_RASR_SRD (MPU_RASR_SRD_Msk &	       \
			     ~(SUB_REGION_0_DISABLED | \
			       SUB_REGION_1_DISABLED | \
			       SUB_REGION_2_DISABLED | \
			       SUB_REGION_3_DISABLED))
#elif (CONFIG_ICCM_MEMORY_SIZE == MEM_SIZE_320K)
#define PSE_ICCM_RASR_SRD (MPU_RASR_SRD_Msk &	       \
			     ~(SUB_REGION_0_DISABLED | \
			       SUB_REGION_1_DISABLED | \
			       SUB_REGION_2_DISABLED | \
			       SUB_REGION_3_DISABLED | \
			       SUB_REGION_4_DISABLED))
#elif (CONFIG_ICCM_MEMORY_SIZE == MEM_SIZE_384K)
#define PSE_ICCM_RASR_SRD (MPU_RASR_SRD_Msk &	       \
			     ~(SUB_REGION_0_DISABLED | \
			       SUB_REGION_1_DISABLED | \
			       SUB_REGION_2_DISABLED | \
			       SUB_REGION_3_DISABLED | \
			       SUB_REGION_4_DISABLED | \
			       SUB_REGION_5_DISABLED))
#else
#error "Unsupported MPU Regison size for ICCM region"
#endif

#if (!(CONFIG_DCCM_MEMORY_SIZE % (MEM_SIZE_64K)) && \
	(CONFIG_DCCM_MEMORY_SIZE <= (MEM_SIZE_384K)))
#define PSE_DCCM_RASR_SRD ((~(PSE_ICCM_RASR_SRD) & MPU_RASR_SRD_Msk) | \
			     SUB_REGION_6_DISABLED | SUB_REGION_7_DISABLED)
#else
#error "Unsupported MPU Regison size for DCCM region"
#endif

#if (CONFIG_L2SRAM_MEMORY_SIZE == MEM_SIZE_1K)
#define L2SRAM_SIZE REGION_1K
#elif (CONFIG_L2SRAM_MEMORY_SIZE == MEM_SIZE_2K)
#define L2SRAM_SIZE REGION_2K
#elif (CONFIG_L2SRAM_MEMORY_SIZE == MEM_SIZE_4K)
#define L2SRAM_SIZE REGION_4K
#elif (CONFIG_L2SRAM_MEMORY_SIZE == MEM_SIZE_8K)
#define L2SRAM_SIZE REGION_8K
#elif (CONFIG_L2SRAM_MEMORY_SIZE == MEM_SIZE_16K)
#define L2SRAM_SIZE REGION_16K
#elif (CONFIG_L2SRAM_MEMORY_SIZE == MEM_SIZE_32K)
#define L2SRAM_SIZE REGION_32K
#elif (CONFIG_L2SRAM_MEMORY_SIZE == MEM_SIZE_64K)
#define L2SRAM_SIZE REGION_64K
#elif (CONFIG_L2SRAM_MEMORY_SIZE == MEM_SIZE_128K)
#define L2SRAM_SIZE REGION_128K
#elif (CONFIG_L2SRAM_MEMORY_SIZE == MEM_SIZE_256K)
#define L2SRAM_SIZE REGION_256K
#elif (CONFIG_L2SRAM_MEMORY_SIZE == MEM_SIZE_512K)
#define L2SRAM_SIZE REGION_512K
#elif (CONFIG_L2SRAM_MEMORY_SIZE == MEM_SIZE_1M)
#define L2SRAM_SIZE REGION_1M
#else
#endif

#if (CONFIG_AONRF_MEMORY_SIZE == MEM_SIZE_1K)
#define PSE_AONRF_SIZE REGION_1K
#elif (CONFIG_AONRF_MEMORY_SIZE == MEM_SIZE_2K)
#define PSE_AONRF_SIZE REGION_2K
#elif (CONFIG_AONRF_MEMORY_SIZE == MEM_SIZE_4K)
#define PSE_AONRF_SIZE REGION_4K
#elif (CONFIG_AONRF_MEMORY_SIZE == MEM_SIZE_8K)
#define PSE_AONRF_SIZE REGION_8K
#elif (CONFIG_AONRF_MEMORY_SIZE == MEM_SIZE_16K)
#define PSE_AONRF_SIZE REGION_16K
#elif (CONFIG_AONRF_MEMORY_SIZE == MEM_SIZE_32K)
#define PSE_AONRF_SIZE REGION_32K
#elif (CONFIG_AONRF_MEMORY_SIZE == MEM_SIZE_64K)
#define PSE_AONRF_SIZE REGION_64K
#elif (CONFIG_AONRF_MEMORY_SIZE == MEM_SIZE_128K)
#define PSE_AONRF_SIZE REGION_128K
#elif (CONFIG_AONRF_MEMORY_SIZE == MEM_SIZE_256K)
#define PSE_AONRF_SIZE REGION_256K
#elif (CONFIG_AONRF_MEMORY_SIZE == MEM_SIZE_512K)
#define PSE_AONRF_SIZE REGION_512K
#elif (CONFIG_AONRF_MEMORY_SIZE == MEM_SIZE_1M)
#define PSE_AONRF_SIZE REGION_1M
#else
#endif
#endif /* _ARM_MPU_MEM_CFG_H_ */
