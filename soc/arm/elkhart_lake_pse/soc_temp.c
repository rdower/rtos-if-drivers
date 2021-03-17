/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*Note: This will be a temp file for all HW/PSS work around and
 * need to remove later.
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>

#include <arch/cpu.h>
#include <arch/arm/aarch32/cortex_m/cmsis.h>
#include <sedi.h>
#include <dashboard_reg.h>
#include <pse_hw_defs.h>
#include <soc_temp.h>

#define LOG_LEVEL CONFIG_SOC_TEMP_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(soc_temp);

#define read32(addr)            (*(volatile uint32_t *)(addr))
#define write32(addr, val)      (*(volatile uint32_t *)(addr) = (uint32_t)(val))

extern void z_NmiHandlerSet(void (*pHandler)(void));

static void soc_nmi_handler(void)
{
	LOG_DBG("%s\n", __func__);

	fw_info_t *const sb = (fw_info_t *)SNOWBALL;
	uint32_t nmi_stt = read32(MISC_NMI_STATUS);
	uint32_t err_no = read32(DASHBOARD_VRF_ERR_STS) & ERRNO_MASK;

	err_no |= (nmi_stt << NMI_STT_SHIFT) + ERRNO_NMI;
	sb->fw_status |= STS_NMI;

}

static void fabric_isr(void)
{
	uint32_t sts0_hbw_lower;
	uint32_t sts0_per0_lower;
	uint32_t arm_axim_iocp_ia_agent_status_lower;
	uint32_t fusa_ctrl;
	uint32_t val;

	fusa_ctrl = read32(FUSA_CTRL_REG);
	if (!(fusa_ctrl & PARITY_LOGIC_EN)) {
		write32(FUSA_CTRL_REG, fusa_ctrl | PARITY_LOGIC_EN);
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();

		while(read32(FUSA_HBW_FABRIC_PARITY_LOG) & (BIT(24)-1))
		    ;
		while(read32(FUSA_PER0_FABRIC_PARITY_LOG) & (BIT(24)-1))
		    ;
		while(read32(FUSA_PER1_FABRIC_PARITY_LOG) & (BIT(24)-1))
		    ;
	}

	sts0_hbw_lower = read32(STATUS0_HBW_LOWER);
	sts0_per0_lower = read32(STATUS0_PER0_LOWER);
	LOG_ERR("Fabric Error: sts0_hbw_lower=0x%x, sts0_per0_lower=0x%x\n",
		(uint32_t)sts0_hbw_lower, (uint32_t)sts0_per0_lower);

	if (sts0_hbw_lower & ARM_AXI_INBAND_ERR) {
		LOG_ERR("ARM_AXIM_IOCP_IA_ERROR_LOG_ADDR_LOWER: %x\n ",
			(uint32_t)read32(ARM_AXIM_IOCP_IA_ERROR_LOG_ADDR_LOWER));

		arm_axim_iocp_ia_agent_status_lower =
			read32(ARM_AXIM_IOCP_IA_AGENT_STATUS_LOWER);
		write32(ARM_AXIM_IOCP_IA_AGENT_STATUS_LOWER,
			arm_axim_iocp_ia_agent_status_lower | INBAND_PRIMARY_ERROR);

		read32(ARM_AXIM_IOCP_IA_AGENT_STATUS_LOWER);
	}
	if (sts0_hbw_lower & ARM_IOSF2AXI_INBAND_ERR) {
		LOG_ERR("IOSF2AXI_IAXI_IA_ERR_LOG_L, %x\n",
			(uint32_t)read32(IOSF2AXI_IAXI_IA_ERR_LOG_L));
		LOG_ERR("IOSF2AXI_IAXI_IA_ERR_LOG_H, %x\n",
			(uint32_t)read32(IOSF2AXI_IAXI_IA_ERR_LOG_H));

		val = read32(IOSF2AXI_IAXI_IA_AGENT_STATUS);
		LOG_ERR("IOSF2AXI_IAXI_IA_AGENT_STATUS, %x\n", (uint32_t)val);
		write32(IOSF2AXI_IAXI_IA_AGENT_STATUS,
			(val | INBAND_PRIMARY_ERROR));
	}

	if (sts0_per0_lower & PER0_INBAND_ERR) {
		LOG_ERR("PER0_FABRIC_IOCP_IA_ERROT_LOG_L, %x\n",
			(uint32_t)read32(PER0_FABRIC_IOCP_IA_ERROT_LOG_LOWER));
		LOG_ERR("PER0_FABRIC_IOCP_IA_ERROT_LOG_H, %x\n",
			(uint32_t)read32(PER0_FABRIC_IOCP_IA_ERROT_LOG_UPPER));

		LOG_ERR("PER0_FABRIC_IOCP_IA_ERROT_LOG_ADDR, %x\n",
			(uint32_t)read32(PER0_FABRIC_IOCP_IA_ERROT_LOG_ADDR));

		val = read32(PER0_FABRIC_IOCP_IA_AGENT_STATUS);
		LOG_ERR("PER0_FABRIC_IOCP_IA_AGENT_STATUS, %x\n", (uint32_t)val);
		write32(PER0_FABRIC_IOCP_IA_AGENT_STATUS,
			(val  | INBAND_PRIMARY_ERROR));
	}

	irq_disable(SOC_HBW_PER_FABRIC_IRQ);
}

int soc_temp_init(void)
{
	LOG_DBG("%s\n", __func__);

	z_NmiHandlerSet(soc_nmi_handler);

	IRQ_CONNECT(SOC_HBW_PER_FABRIC_IRQ,
		    SOC_FABRIC_IRQ_PRI, fabric_isr,
		    NULL, 0);

	irq_enable(SOC_HBW_PER_FABRIC_IRQ);


	return 0;
}
