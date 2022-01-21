/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr.h>
#include <init.h>

#include <arch/arm/aarch32/cortex_m/mpu/arm_mpu.h>
#include "arm_mpu_mem_cfg.h"
#if defined(CONFIG_ENABLE_HW_PSS_WORKAROUND)
#include "soc_temp.h"
#endif

#define LOG_LEVEL CONFIG_MPU_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(arm_mpu_regions);


#define REGION_RWEX_ATTR(size)						 \
	{								 \
		(NORMAL_OUTER_INNER_WRITE_THROUGH_NON_SHAREABLE | size | \
		 P_RW_U_NA_Msk)						 \
	}

#define REGION_PPB_XN_ATTR(size) { (STRONGLY_ORDERED_SHAREABLE | size |	\
				    MPU_RASR_XN_Msk | P_RW_U_NA_Msk) }

extern char _ram_ro_start[];
extern char _ram_ro_end[];

extern char _l2sram_mpu_ro_region_end[];
extern char __l2sram_text_start[];

static struct arm_mpu_region mpu_regions[] = {
	MPU_REGION_ENTRY("BACK_NA", ARM_ADDRESS_SPACE,
			 REGION_NO_ACCESS_ATTR(REGION_4G)),

	MPU_REGION_ENTRY("PERIPHERAL", IO_ADDRESS_SPACE,
			 REGION_PPB_XN_ATTR(REGION_512M)),

	MPU_REGION_ENTRY("AONRF", CONFIG_AONRF_MEMORY_BASE_ADDRESS,
			 REGION_RWEX_ATTR(PSE_AONRF_SIZE)),

#if  defined(CONFIG_EN_MPU_L2SRAM_MEM_ACCESS)
	MPU_REGION_ENTRY("L2SRAM", L2SRAM_MEM_START_ADDR,
			 REGION_RAM_ATTR(REGION_1M)),
#endif

#if  defined(CONFIG_EN_MPU_ICCM_MEM_ACCESS)
	MPU_REGION_ENTRY("ICCM", ICCM_MEM_START_ADDR,
			 REGION_FLASH_ATTR(REGION_512K | PSE_ICCM_RASR_SRD)),
#endif

#if  defined(CONFIG_EN_MPU_DCCM_MEM_ACCESS)
	MPU_REGION_ENTRY("DCCM", DCCM_MEM_START_ADDR,
			 REGION_RAM_ATTR(REGION_512K | PSE_DCCM_RASR_SRD)),
#endif

#if defined(CONFIG_EN_L2SRAM_RELOCATION_MEM)
#if defined(CONFIG_EN_MPU_L2SRAM_MEM_ACCESS)
	MPU_REGION_ENTRY("SRAM_RO", CONFIG_L2SRAM_MEMORY_BASE_ADDRESS,
			 REGION_FLASH_ATTR(REGION_2G)),
#else
#error "Please enable L2SRAM memory if you need to relocate to L2SRAM!!!"
#endif
#else
	MPU_REGION_ENTRY("SRAM_RO", CONFIG_SRAM_BASE_ADDRESS,
			 REGION_FLASH_ATTR(REGION_2G)),
#endif
};

struct arm_mpu_config mpu_config = {
	.num_regions = ARRAY_SIZE(mpu_regions), .mpu_regions = mpu_regions,
};

int mpu_config_init(void)
{
	int nsize, index;
	uint32_t asize;

	LOG_DBG("%s\n", __func__);

#if CONFIG_EN_L2SRAM_RELOCATION_MEM
	uint32_t rsize =
		(uint32_t)(_l2sram_mpu_ro_region_end - __l2sram_text_start);

	LOG_DBG("L2SRAM Relocation text_rodata section size:%x\n", rsize);
#else
	uint32_t rsize = (uint32_t)(_ram_ro_end - _ram_ro_start);

	LOG_DBG("Kernel text_rodata section size:%x\n", rsize);
#endif

	for (nsize = 0;; nsize++) {
		uint32_t size = (uint32_t)0x1 << nsize;

		if (size > CONFIG_SRAM_SIZE * MEM_SIZE_1K) {
			__ASSERT_NO_MSG(0);
			return -EINVAL;
		}

		if (size == (uint32_t)rsize) {
			break;
		}
	}

	asize = ((nsize - 1) << MPU_RASR_SIZE_Pos) & MPU_RASR_SIZE_Msk;
	for (index = 0; index < mpu_config.num_regions; index++) {
		arm_mpu_region_attr_t attr = REGION_FLASH_ATTR(asize);
		arm_mpu_region_attr_t *p = &mpu_config.mpu_regions[index].attr;
		uint32_t _asize = (mpu_config.mpu_regions[index].attr.rasr &
				   MPU_RASR_SIZE_Msk);

		if (_asize != REGION_2G) {
			continue;
		}

		LOG_DBG("base:%x attr:%x\n",
			mpu_config.mpu_regions[index].base,
			mpu_config.mpu_regions[index].attr.rasr);

		*p = attr;
		return 0;
	}

	return -ENOENT;
}

/*
 * SYS_INIT(mpu_config_init, PRE_KERNEL_1,
 *	 CONFIG_KERNEL_INIT_PRIORITY_OBJECTS);
 */
