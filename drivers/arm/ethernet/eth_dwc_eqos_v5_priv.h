/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_ETHERNET_ETH_DWC_EQOS_PRIV_H_
#define DRIVERS_ETHERNET_ETH_DWC_EQOS_PRIV_H_

#ifdef CONFIG_PCI
#include <pci/pci.h>
#include <pci/pci_mgr.h>
#endif /* CONFIG_PCI */

#include <sys/util.h>
#include "dw_tsn_lib.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ONE_SEC_IN_MICROSEC 1000000ULL
#define ONE_SEC_IN_NANOSEC 1000000000ULL
#define NET_BUF_TIMEOUT K_MSEC(100)
#ifdef CONFIG_ETH_DWC_EQOS_RX_NAPI
#define ETH_DWC_EQOS_NAPI_WAIT 5
#endif
#define ETH_DWC_EQOS_MTU 1500
#define NET_FCS_LEN 4

#ifndef CONFIG_NET_VLAN
#define NET_ETHH_LEN sizeof(struct net_eth_hdr)
#else
#define NET_ETHH_LEN sizeof(struct net_eth_vlan_hdr)
#define RDESC2_VLAN_TYPE 4
#endif /* CONFIG_NET_VLAN */

#define ETH_MAX_FRM_SZ (NET_ETHH_LEN + ETH_DWC_EQOS_MTU + NET_FCS_LEN)
#define FULL_DUPLX 1
#define HALF_DUPLX 0
#define LINK_UP 1
#define LINK_DOWN 0

#define POW2(y) ({                     \
	int val = 1;                   \
\
	for (int j = 0; j < y; j++) {  \
		val *= 2;              \
	}                              \
\
	val;                           \
})

#define DEF2STR_2(x) #x
#define DEF2STR(x) DEF2STR_2(x)
#ifdef ETH_DWC_EQOS_MAX_TX_QUEUES
#if (CONFIG_ETH_DWC_EQOS_TX_QUEUES > ETH_DWC_EQOS_MAX_TX_QUEUES)
#define STR_MAX_TXQ DEF2STR(ETH_DWC_EQOS_MAX_TX_QUEUES)
#pragma message "Platform allowed max tx queues: " STR_MAX_TXQ
#error "Kconfig tx queues setting larger than platform allowed"
#endif /* CONFIG_ETH_DWC_EQOS_TX_QUEUES > ETH_DWC_EQOS_MAX_TX_QUEUES */
#endif /* ETH_DWC_EQOS_MAX_TX_QUEUES */

#ifdef ETH_DWC_EQOS_MAX_RX_QUEUES
#if (CONFIG_ETH_DWC_EQOS_RX_QUEUES > ETH_DWC_EQOS_MAX_RX_QUEUES)
#define STR_MAX_RXQ DEF2STR(ETH_DWC_EQOS_MAX_RX_QUEUES)
#pragma message "Platform allowed max rx queues: " STR_MAX_RXQ
#error "Kconfig rx queues setting larger than platform allowed"
#endif /* CONFIG_ETH_DWC_EQOS_RX_QUEUES > ETH_DWC_EQOS_MAX_RX_QUEUES */
#endif /* ETH_DWC_EQOS_MAX_RX_QUEUES */

#if (defined(ETH_DWC_EQOS_0_TX0_IRQ) || defined(ETH_DWC_EQOS_1_TX0_IRQ)) \
	&& defined(CONFIG_ETH_DWC_EQOS_IRQ_MODE)
#define ETH_DWC_EQOS_MULTI_IRQ
#endif

/* Cache alignment */
#define ETH_DWC_EQOS_DCACHE_ALIGNMENT		32

/* GPTP rate ratio limits */
#define ETH_DWC_EQOS_GPTP_RATE_RATIO_UPPER_LIMIT 1.01
#define ETH_DWC_EQOS_GPTP_RATE_RATIO_LOWER_LIMIT 0.99

enum eth_port_id {
	ETH_PORT_0,
	ETH_PORT_1,
};

enum device_flags {
	ETH_DEV_FLAGS_PCI_DETECTED              = 0x00000001,
	ETH_DEV_FLAGS_TX_CSUM_OFFLOAD           = 0x00000002,
	ETH_DEV_FLAGS_RX_CSUM_OFFLOAD           = 0x00000004,
	ETH_DEV_FLAGS_TSSEL                     = 0x00000008,
	ETH_DEV_FLAGS_QAV                       = 0x00000010,
	ETH_DEV_FLAGS_QBV                       = 0x00000020,
	ETH_DEV_FLAGS_QBU                       = 0x00000040,
	ETH_DEV_FLAGS_TBS                       = 0x00000080,
};

typedef void (*eth_config_irq_t)(const struct device *port);

struct eth_config {
	eth_config_irq_t config_func;

#ifdef CONFIG_ETH_DWC_EQOS_SHARED_IRQ
	char *sharedirq_devname;
#endif /* CONFIG_ETH_DWC_EQOS_SHARED_IRQ */
};

#define TX_DESC_CHECKSUM_DIS 0
#define TX_DESC_CHECKSUM_EN 3
/* Transmit descriptor */
struct eth_tx_desc {
	/* First word of descriptor */
	union {
		/* TX descriptor read format */
		/* Pointer to frame data buffer */
		uint8_t *buf1ap;
		/* TX descriptor write-back format & TX context descriptor
		 * format
		 */
		uint32_t timestamp_low;
		uint32_t dword;
	} tdes0;
	/* Second word of descriptor */
	union {
		/* TX descriptor read format */
		/* Unused, since this driver initializes only a single
		 * descriptor for each direction.
		 */
		uint8_t *buf2ap;
		/* TX descriptor write-back format & TX context descriptor
		 * format
		 */
		uint32_t timestamp_high;
		uint32_t dword;
	} tdes1;
	/* Third word of descriptor */
	union {
		/* TX descriptor read format */
		struct {
			uint32_t hl_bl1            : 14;
			uint32_t vtir              : 2;
			uint32_t bl2               : 14;
			uint32_t ttse_tmwd         : 1;
			uint32_t ioc               : 1;
		} rd;
		/* TX context descriptor format */
		struct {
			uint32_t mss               : 14;
			uint32_t rsvd              : 2;
			uint32_t ivt               : 16;
		} ctxt;
		uint32_t dword;
	} tdes2;
	/* Forth word of descriptor */
	union {
		/* TX descriptor read format */
		struct {
			uint32_t fl                : 15;
			uint32_t rsvd              : 1;
			uint32_t cic               : 2;
			uint32_t tse               : 1;
			uint32_t slotnum_thl       : 4;
			uint32_t saic              : 3;
			uint32_t cpc               : 2;
			uint32_t ld                : 1;
			uint32_t fd                : 1;
			uint32_t ctxt              : 1;
			uint32_t own               : 1;
		} rd;
		/* TX descriptor write-back format */
		struct {
			uint32_t ihe               : 1;
			uint32_t db                : 1;
			uint32_t uf                : 1;
			uint32_t ed                : 1;
			uint32_t cc                : 4;
			uint32_t ec                : 1;
			uint32_t lc                : 1;
			uint32_t nc                : 1;
			uint32_t loc               : 1;
			uint32_t pce               : 1;
			uint32_t ff                : 1;
			uint32_t jt                : 1;
			uint32_t es                : 1;
			uint32_t eue               : 1;
			uint32_t ttss              : 1;
			uint32_t rsvd              : 10;
			uint32_t ld                : 1;
			uint32_t fd                : 1;
			uint32_t ctxt              : 1;
			uint32_t own               : 1;
		} wb;
		/* TX context descriptor format */
		struct {
			uint32_t vt                : 16;
			uint32_t vltv              : 1;
			uint32_t ivltv             : 1;
			uint32_t rsvd1             : 5;
			uint32_t de                : 1;
			uint32_t rsvd2             : 2;
			uint32_t tcmssv            : 1;
			uint32_t ostc              : 1;
			uint32_t rsvd3             : 2;
			uint32_t ctxt              : 1;
			uint32_t own               : 1;
		} ctxt;
		uint32_t dword;
	} tdes3;
};

#define ENH_TX_DESC_LT_SEC_MASK 0xFF
#define ENH_TX_DESC_LT_SEC_SHIFT 32
#define ENH_TX_DESC_LT_NANOSEC_MASK 0xFFFFFF00
#define ENH_TX_DESC_LT_NANOSEC_SHIFT 8
/* Enhanced Transmit descriptor */
struct enh_eth_tx_desc {
	/* First word of enhanced descriptor */
	union {
		/* Enhanced descriptor read & write-back format */
		struct {
			uint32_t lt_s              : 8;
			uint32_t gsn               : 4;
			uint32_t rsvd              : 19;
			uint32_t ltv               : 1;
		};
		uint32_t dword;
	} etdes4;
	/* Second word of enhanced descriptor */
	union {
		/* Enhanced descriptor read & write-back format */
		struct {
			uint32_t rsvd              : 8;
			uint32_t lt_ns             : 24;
		};
		uint32_t dword;
	} etdes5;
	/* Third word of enhanced descriptor */
	union {
		/* Reserved in enhanced descriptor */
		uint32_t dword;
	} etdes6;
	/* Forth word of enhanced descriptor */
	union {
		/* Reserved in enhanced descriptor */
		uint32_t dword;
	} etdes7;
	/* First word of descriptor */
	union {
		/* TX descriptor read format */
		/* Pointer to frame data buffer */
		uint8_t *buf1ap;
		/* TX descriptor write-back format & TX context descriptor
		 * format
		 */
		uint32_t timestamp_low;
		uint32_t dword;
	} tdes0;
	/* Second word of descriptor */
	union {
		/* TX descriptor read format */
		/* Unused, since this driver initializes only a single
		 * descriptor for each direction.
		 */
		uint8_t *buf2ap;
		/* TX descriptor write-back format & TX context descriptor
		 * format
		 */
		uint32_t timestamp_high;
		uint32_t dword;
	} tdes1;
	/* Third word of descriptor */
	union {
		/* TX descriptor read format */
		struct {
			uint32_t hl_bl1            : 14;
			uint32_t vtir              : 2;
			uint32_t bl2               : 14;
			uint32_t ttse_tmwd         : 1;
			uint32_t ioc               : 1;
		} rd;
		/* TX context descriptor format */
		struct {
			uint32_t mss               : 14;
			uint32_t rsvd              : 2;
			uint32_t ivt               : 16;
		} ctxt;
		uint32_t dword;
	} tdes2;
	/* Forth word of descriptor */
	union {
		/* TX descriptor read format */
		struct {
			uint32_t fl                : 15;
			uint32_t rsvd              : 1;
			uint32_t cic               : 2;
			uint32_t tse               : 1;
			uint32_t slotnum_thl       : 4;
			uint32_t saic              : 3;
			uint32_t cpc               : 2;
			uint32_t ld                : 1;
			uint32_t fd                : 1;
			uint32_t ctxt              : 1;
			uint32_t own               : 1;
		} rd;
		/* TX descriptor write-back format */
		struct {
			uint32_t ihe               : 1;
			uint32_t db                : 1;
			uint32_t uf                : 1;
			uint32_t ed                : 1;
			uint32_t cc                : 4;
			uint32_t ec                : 1;
			uint32_t lc                : 1;
			uint32_t nc                : 1;
			uint32_t loc               : 1;
			uint32_t pce               : 1;
			uint32_t ff                : 1;
			uint32_t jt                : 1;
			uint32_t es                : 1;
			uint32_t eue               : 1;
			uint32_t ttss              : 1;
			uint32_t rsvd              : 10;
			uint32_t ld                : 1;
			uint32_t fd                : 1;
			uint32_t ctxt              : 1;
			uint32_t own               : 1;
		} wb;
		/* TX context descriptor format */
		struct {
			uint32_t vt                : 16;
			uint32_t vltv              : 1;
			uint32_t ivltv             : 1;
			uint32_t rsvd1             : 5;
			uint32_t de                : 1;
			uint32_t rsvd2             : 2;
			uint32_t tcmssv            : 1;
			uint32_t ostc              : 1;
			uint32_t rsvd3             : 2;
			uint32_t ctxt              : 1;
			uint32_t own               : 1;
		} ctxt;
		uint32_t dword;
	} tdes3;
};

typedef struct enh_eth_tx_desc enh_desc_t;

/* Receive descriptor */
struct eth_rx_desc {
	/* First word of descriptor */
	union {
		/* RX descriptor read format */
		/* Pointer to frame receive buffer */
		uint8_t *buf1ap;
		/* RX descriptor write-back format */
		struct {
			uint32_t ovt               : 16;
			uint32_t ivt               : 16;
		};
		/* RX context descriptor format */
		uint32_t timestamp_low;
		uint32_t dword;
	} rdes0;
	/* Second word of descriptor */
	union {
		/* RX descriptor write-back format */
		struct {
			uint32_t pt                : 3;
			uint32_t iphe              : 1;
			uint32_t ipv4              : 1;
			uint32_t ipv6              : 1;
			uint32_t ipcb              : 1;
			uint32_t ipce              : 1;
			uint32_t pmt               : 4;
			uint32_t pft               : 1;
			uint32_t pv                : 1;
			uint32_t tsa               : 1;
			uint32_t td                : 1;
			uint32_t opc               : 16;
		};
		/* RX context descriptor format */
		uint32_t timestamp_high;
		uint32_t dword;
	} rdes1;
	/* Third word of descriptor */
	union {
		/* RX descriptor read format */
		/* Unused, since this driver initializes only a single
		 * descriptor for each direction.
		 */
		uint8_t *buf2ap;
		/* RX descriptor write-back format */
		struct {
			uint32_t hl                : 10;
			uint32_t arpnr             : 1;
			uint32_t rsvd              : 3;
			uint32_t its               : 1;
			uint32_t ots               : 1;
			uint32_t saf               : 1;
			uint32_t daf               : 1;
			uint32_t hf                : 1;
			uint32_t madrm             : 8;
			uint32_t l3fm              : 1;
			uint32_t l4fm              : 1;
			uint32_t l3l4fm            : 3;
		};
		uint32_t dword;
	} rdes2;
	/* Forth word of descriptor */
	union {
		/* RX descriptor read format */
		struct {
			uint32_t rsvd1             : 24;
			uint32_t buf1v             : 1;
			uint32_t buf2v             : 1;
			uint32_t rsvd2             : 4;
			uint32_t ioc               : 1;
			uint32_t own               : 1;
		} rd;
		/* RX descriptor write-back format */
		struct {
			uint32_t pl                : 15;
			uint32_t es                : 1;
			uint32_t lt                : 3;
			uint32_t de                : 1;
			uint32_t re                : 1;
			uint32_t oe                : 1;
			uint32_t rwt               : 1;
			uint32_t gp                : 1;
			uint32_t ce                : 1;
			uint32_t rs0v              : 1;
			uint32_t rs1v              : 1;
			uint32_t rs2v              : 1;
			uint32_t ld                : 1;
			uint32_t fd                : 1;
			uint32_t ctxt              : 1;
			uint32_t own               : 1;
		} wb;
		/* RX context descriptor format */
		struct {
			uint32_t rsvd              : 29;
			uint32_t de                : 1;
			uint32_t ctxt              : 1;
			uint32_t own               : 1;
		} ctxt;
		uint32_t dword;
	} rdes3;
};

struct dwc_eqos_stats {
	net_stats_t rx_pkt_n[CONFIG_ETH_DWC_EQOS_RX_QUEUES];
};

/* Driver metadata associated with each Ethernet device */
struct eth_runtime {
	uint32_t port_id;
	int (*get_platdata)(const struct device *port);
	uint32_t flags;
#ifdef CONFIG_ETH_DWC_EQOS_POLLING_MODE
	struct k_timer polling_timer;
#endif /* CONFIG_ETH_DWC_EQOS_POLLING_MODE */
#ifdef CONFIG_ETH_PHY_POLLING_MODE
	struct k_timer phy_polling_timer;
#endif /* CONFIG_ETH_PHY_POLLING_MODE */
	uint8_t addrcnt;
	uint8_t hashtblsz;
	uint8_t vlancnt;
	uint8_t txqnum;
	uint8_t rxqnum;
	uint32_t txfifosz;
	uint32_t rxfifosz;
	uint32_t base_addr;
#ifdef CONFIG_ETH_DWC_EQOS_NETWORK_PROXY
	uint32_t np_misc_base_addr;
	uint32_t np_att_base_addr;
	uint32_t np_shmem_addr;
	uint32_t np_shmem_size;
	uint32_t np_vnn_id;
	struct k_work np_work;
#endif
	uint32_t irq_num;
	uint32_t mdio_csr_clk;
	bool autoneg;
#ifdef CONFIG_ETH_DWC_EQOS_NETWORK_PROXY
	bool	netprox;
#endif
	int32_t link_speed;
	int8_t duplex_mode;
	uint8_t link_status;
	bool use_xpcs;
	bool use_eee;
	uint32_t lpi_timer;
	struct phy_device phy_dev;
	struct k_work phy_work;
	struct net_if *iface;
#ifdef CONFIG_ETH_DWC_EQOS_FLEX_RX_PARSER
	uint32_t frp_entry_sz;
	uint32_t frp_buf_sz;
#endif /* CONFIG_ETH_DWC_EQOS_FLEX_RX_PARSER */
#ifdef CONFIG_NET_IPV6
	struct net_if_mcast_monitor mcast_mon;
#endif /* CONFIG_NET_IPV6 */
#ifdef CONFIG_NET_PKT_TIMESTAMP
	uint64_t ptp_clock_rate;
	uint32_t default_addend;
#endif /* CONFIG_NET_PKT_TIMESTAMP */
#ifdef CONFIG_ETH_DWC_EQOS_PTP
	const struct device *ptp_clock;
#endif /* CONFIG_ETH_DWC_EQOS_PTP */
#ifdef CONFIG_ETH_DWC_EQOS_PCI
	struct pci_dev_info pci_dev;
#endif  /* CONFIG_ETH_DWC_EQOS_PCI */
#ifdef CONFIG_NET_STATISTICS_ETHERNET
	struct net_stats_eth stats;
#ifdef CONFIG_NET_STATISTICS_ETHERNET_VENDOR
	struct dwc_eqos_stats eth_stats;
#endif /* CONFIG_NET_STATISTICS_ETHERNET_VENDOR */
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
	/* Transmit descriptor */
#define RING_SZ CONFIG_ETH_DWC_EQOS_DMA_RING_SIZE
#define MAX_TXQ CONFIG_ETH_DWC_EQOS_TX_QUEUES
	volatile struct eth_tx_desc tx_desc[MAX_TXQ][RING_SZ] __aligned(32);
#ifdef CONFIG_ETH_DWC_EQOS_TBS
	volatile enh_desc_t enh_tx_desc[MAX_TXQ][RING_SZ] __aligned(32);
	bool tbs_enabled[MAX_TXQ];
#endif
	struct k_sem txq_lock[MAX_TXQ];
	struct k_work tx_irq_work[MAX_TXQ];
	int tdesc_ring_wr_ptr[MAX_TXQ];
	int tdesc_ring_rd_ptr[MAX_TXQ];
	struct net_pkt *tdesc_pkt[MAX_TXQ][RING_SZ];
	struct net_buf *tdesc_frag[MAX_TXQ][RING_SZ];
	/* Receive descriptor */
#define MAX_RXQ CONFIG_ETH_DWC_EQOS_RX_QUEUES
	volatile struct eth_rx_desc rx_desc[MAX_RXQ][RING_SZ] __aligned(32);
	struct k_sem rxq_lock[MAX_RXQ];
	struct k_delayed_work rx_irq_work[MAX_RXQ];
#ifdef CONFIG_ETH_DWC_EQOS_RX_NAPI
	int rx_napi_count[MAX_RXQ];
	bool rx_napi_force_stop;
#endif
	int rdesc_ring_wr_ptr[MAX_RXQ];
	int rdesc_ring_rd_ptr[MAX_RXQ];
	/* Receive DMA packet buffer */
	volatile uint8_t rx_buf[MAX_RXQ][RING_SZ][ETH_MAX_FRM_SZ] __aligned(32);

	union {
		struct {
			uint8_t bytes[6];
			uint8_t pad[2];
		};
		uint32_t words[2];
	} mac_addr;
#ifdef CONFIG_ETH_DWC_EQOS_QAV
	struct cbs_param cbsparam[MAX_TXQ - 1];
#endif
#ifdef CONFIG_ETH_DWC_EQOS_QBV
	struct est_param estparam;
	struct k_work tsn_work;
#endif
#ifdef CONFIG_ETH_DWC_EQOS_QBU
	struct fpe_param fpeparam;
#endif
};

/*
 * MAC Registers
 */
#define MAC_CONFIGURATION                       0x0000
#define MAC_CONF_ARP                            BIT(31)
#define MAC_CONF_IPC                            BIT(27)
#define MAC_CONF_CST                            BIT(21)
#define MAC_CONF_ACS                            BIT(20)
#define INV_MAC_CONF_SPD                        0xFFFF3FFF
#define MAC_CONF_SPD_10MHZ                      0x00008000
#define MAC_CONF_SPD_100MHZ                     0x0000C000
#define MAC_CONF_SPD_1000MHZ                    0x00000000
#define MAC_CONF_SPD_2500MHZ                    0x00004000
#define MAC_CONF_DM                             BIT(13)
#define MAC_CONF_LM                             BIT(12)
#define MAC_CONF_DO                             BIT(10)
#define MAC_CONF_TE                             BIT(1)
#define MAC_CONF_RE                             BIT(0)

#define MAC_PACKET_FILTER                       0x0008
#define MAC_PKT_FLTR_VTFE                       BIT(16)
#define MAC_PKT_FLTR_HPF                        BIT(10)
#define MAC_PKT_FLTR_PM                         BIT(4)
#define MAC_PKT_FLTR_HMC                        BIT(2)
#define MAC_PKT_FLTR_HUC                        BIT(1)
#define MAC_PKT_FLTR_PR                         BIT(0)

#define MAC_HASH_TABLE_REG(x)                   (0x0010 + (x * 0x4))
#define MAC_HASH_VAL_MASK                       0x1F
#define MAC_HASH_64_VAL_SHIFT                   2
#define MAC_HASH_128_VAL_SHIFT                  1
#define MAC_HASH_64_REG_SEL_BIT                 BIT(5)
#define MAC_HASH_128_REG_SEL_BIT                (BIT(5) | BIT(6))
#define MAC_HASH_256_REG_SEL_BIT                (BIT(5) | BIT(6) | BIT(7))

#define MAC_VLAN_TAG_CTRL                       0x0050
#define MAC_VLAN_TAG_CTRL_EVLRXS                BIT(24)
#define MAC_VLAN_TAG_CTRL_EVLS_IFPASS           BIT(21)
#define MAC_VLAN_TAG_CTRL_EVLS_ALWY             (BIT(22) | BIT(21))
#define MAC_VLAN_TAG_CTRL_OFS_MASK              0x0000007C
#define MAC_VLAN_TAG_CTRL_OFS_SHIFT             2
#define MAC_VLAN_TAG_CTRL_CT_RD                 BIT(1)
#define MAC_VLAN_TAG_CTRL_CT_WR_INVMSK          0xFFFFFFFD
#define MAC_VLAN_TAG_CTRL_OB                    BIT(0)
#define MAC_VLAN_TAG_FILT                       0x0054
#define MAC_VLAN_TAG_FILT_DMACHN_MASK           0x0E000000
#define MAC_VLAN_TAG_FILT_DMACHN_SHIFT          25
#define MAC_VLAN_TAG_FILT_DMACHEN               BIT(24)
#define MAC_VLAN_TAG_FILT_ETV                   BIT(17)
#define MAC_VLAN_TAG_FILT_VEN                   BIT(16)
#define MAC_VLAN_TAG_FILT_VID_MASK              0x0000FFFF

#define MAC_RXQ_CTRL0                           0x00A0
#define MAC_RXQ_CTRL2                           0x00A8
#define MAC_RXQ_CTRL3                           0x00AC
#define INV_MAC_DISABLE_RXQ(x)                  ~((BIT(0) | BIT(1)) << (x * 2))
#define MAC_EN_AV_RXQ(x)                        (BIT(0) << (x * 2))
#define MAC_EN_DCB_GEN_RXQ(x)                   (BIT(1) << (x * 2))

/* Priorities setting to route to different queues.
 * For Rx queue number <= 4, configure CTRL2 register.
 * For Rx queue number >= 5 and <= 8, configure CTRL2 and CTRL3 register.
 *
 * An Example of 4 Rx queues setting on CTRL2 register:
 * Q1 set to 0 & 1 priorities; Q2 set to 2 & 3 priorities;
 * Q3 set to 4 & 5 priorities; Q4 set to 6 & 7 priorities.
 *
 *           Q4       Q3       Q2       Q1
 * (bit31)|--------|--------|--------|--------|(bit0)
 *        |11000000|00110000|00001100|00000011|
 */
#define MAC_RXQ_PRIO_CTRL2                      0xC0300C03
#define MAC_RXQ_PRIO_CTRL3                      0

#define MAC_INTERRUPT_STATUS                    0x00B0
#define MAC_INTR_STS_MDIOIS                     BIT(18)
#define MAC_INTR_STS_FPEIS                      BIT(17)
#define MAC_INTR_STS_PHYIS                      BIT(3)
#define MAC_INTR_STS_RGSMIIIS                   BIT(0)
#define MAC_INTERRUPT_ENABLE                    0x00B4
#define MAC_INTR_EN_MDIOIE                      BIT(18)
#define MAC_INTR_EN_FPEIE                       BIT(17)
#define MAC_INTR_EN_RXSTSIE                     BIT(14)
#define MAC_INTR_EN_TXSTSIE                     BIT(13)
#define MAC_INTR_EN_PHYIE                       BIT(3)
#define MAC_INTR_EN_RGSMIIIE                    BIT(0)

#define MAC_VERSION                             0x0110
#define MAC_VERSION_MASK                        0x00FF

#define MAC_HW_FEATURE0                         0x011C
#define MAC_HW_FEAT0_TSSEL                      BIT(12)
#define MAC_HW_FEAT0_TXCOESEL                   BIT(14)
#define MAC_HW_FEAT0_RXCOESEL                   BIT(16)
#define MAC_HW_FEAT0_ADDMACADRSEL_MASK          0x007C0000
#define MAC_HW_FEAT0_ADDMACADRSEL_SHIFT         18
#define MAC_HW_FEATURE1                         0x0120
#define MAC_HW_FEAT1_AVSEL                      BIT(20)
#define MAC_HW_FEAT1_HASHTBLSZ_MASK             0x03000000
#define MAC_HW_FEAT1_HASHTBLSZ_SHIFT            24
#define MAC_HW_FEAT1_HASHTBLSZ_64               0x01
#define MAC_HW_FEAT1_HASHTBLSZ_128              0x02
#define MAC_HW_FEAT1_HASHTBLSZ_256              0x03
#define MAC_HW_FEAT1_TXFIFOSZ_MASK              0x000007C0
#define MAC_HW_FEAT1_TXFIFOSZ_SHIFT             6
#define MAC_HW_FEAT1_RXFIFOSZ_MASK              0x0000001F
#define MAC_HW_FEATURE2                         0x0124
#define MAC_HW_FEAT2_TXCHCNT_MASK               0x003C0000
#define MAC_HW_FEAT2_TXCHCNT_SHIFT              18
#define MAC_HW_FEAT2_RXCHCNT_MASK               0x0000F000
#define MAC_HW_FEAT2_RXCHCNT_SHIFT              12
#define MAC_HW_FEAT2_TXQCNT_MASK                0x000003C0
#define MAC_HW_FEAT2_TXQCNT_SHIFT               6
#define MAC_HW_FEAT2_RXQCNT_MASK                0x0000000F
#define MAC_HW_FEATURE3                         0x0128
#define MAC_HW_FEAT3_TBSSEL                     BIT(27)
#define MAC_HW_FEAT3_FRPES_MASK                 0x00006000
#define MAC_HW_FEAT3_FRPES_SHIFT                13
#define MAC_HW_FEAT3_FRPES_64ENTR               0
#define MAC_HW_FEAT3_FRPES_128ENTR              1
#define MAC_HW_FEAT3_FRPES_256ENTR              2
#define MAC_HW_FEAT3_FRPBS_MASK                 0x00001800
#define MAC_HW_FEAT3_FRPBS_SHIFT                11
#define MAC_HW_FEAT3_FRPBS_64BYTS               0
#define MAC_HW_FEAT3_FRPBS_128BYTS              1
#define MAC_HW_FEAT3_FRPBS_256BYTS              2
#define MAC_HW_FEAT3_FRPSEL                     BIT(10)
#define MAC_HW_FEAT3_NRVF_MASK                  0x00000007
#define MAC_HW_FEAT3_NRVF_4                     0x1
#define MAC_HW_FEAT3_NRVF_8                     0x2
#define MAC_HW_FEAT3_NRVF_16                    0x3
#define MAC_HW_FEAT3_NRVF_24                    0x4
#define MAC_HW_FEAT3_NRVF_32                    0x5

#define MAC_LPI_CONTROL_STATE                   0x00D0
#define MAC_LPI_CONTROL_STATE_LPITCSE           BIT(21)
#define MAC_LPI_CONTROL_STATE_LPIATE            BIT(20)
#define MAC_LPI_CONTROL_STATE_LPITXA            BIT(19)
#define MAC_LPI_CONTROL_STATE_PLS               BIT(17)
#define MAC_LPI_CONTROL_STATE_LPIEN             BIT(16)
#define MAC_LPI_CONTROL_STATE_TLPIEN            BIT(0)

#define MAC_LPI_TIMERS_CONTROL                  0x00D4
#define MAC_LPI_TIMERS_CONTROL_LST_DEFAULT      0x3E8
#define MAC_LPI_TIMERS_CONTROL_LST_MASK         0x3FF
#define MAC_LPI_TIMERS_CONTROL_LST_SHIFT        16
#define MAC_LPI_TIMERS_CONTROL_TWT_DEFAULT      0x64
#define MAC_LPI_TIMERS_CONTROL_TWT_MASK         0xFFFF

#define MAC_LPI_ENTRY_TIMER                     0x00D8
#define MAC_LPI_ENTRY_TIMER_START_MASK          0xFFFF8
#define MAC_1US_TIC_CNTR                        0x00DC
#define MAC_1US_TIC_CNTR_VALUE(x)               (((x) - 1000000) / 1000000)
#define MAC_1US_TIC_CNTR_MASK                   0xFFF
#define MAC_TX_LPI_CNTR                         0x7F0
#define MAC_RX_LPI_CNTR                         0x7F8

#define MAC_PHYIF_CONTROL_STATUS                0x00F8

#define MAC_MDIO_ADDRESS                        0x0200
#define MAC_MDIO_PA_MASK                        0x03E00000
#define MAC_MDIO_PA_SHIFT                       21
#define MAC_MDIO_RDA_MASK                       0x001F0000
#define MAC_MDIO_RDA_SHIFT                      16
#define MAC_MDIO_DATA                           0x0204
#define MAC_MDIO_RA_MASK                        0xFFFF0000
#define MAC_MDIO_RA_SHIFT                       16
#define MAC_MDIO_GMII_BUSY                      BIT(0)
#define MAC_MDIO_CLAUSE_45_PHY_EN               BIT(1)
#define MAC_MDIO_GMII_OPR_CMD_READ              0x0000000C
#define MAC_MDIO_GMII_OPR_CMD_WRITE             0x00000004
#define INV_MAC_MDIO_CSR_CLOCK                  0xFFFFF0FF
#define MAC_MDIO_CSR_CLOCK_60_100MHZ            0x00000000
#define MAC_MDIO_CSR_CLOCK_100_150MHZ           0x00000100
#define MAC_MDIO_CSR_CLOCK_20_35MHZ             0x00000200
#define MAC_MDIO_CSR_CLOCK_35_60MHZ             0x00000300
#define MAC_MDIO_CSR_CLOCK_150_250MHZ           0x00000400
#define MAC_MDIO_CSR_CLOCK_250_300MHZ           0x00000500
#define MAC_MDIO_CSR_CLOCK_300_500MHZ           0x00000600
#define MAC_MDIO_CSR_CLOCK_500_800MHZ           0x00000700

#define MAC_GPIO_STATUS                         0x020C
#define MAC_GPIO_STS_GPO_PTP_MASK               0x00090000
#define MAC_GPIO_STS_GPO_PTP_256_100MHZ         0
#define MAC_GPIO_STS_GPO_PTP_19_2MHz            BIT(16)
#define MAC_GPIO_STS_GPO_PTP_200MHZ             (BIT(16) | BIT(19))

#define MAC_ARP_ADDRESS                         0x210

#define MAC_ADDRESS_LOW(x)                      (0x0304 + (x * 0x8))
#define MAC_ADDRESS_HIGH(x)                     (0x0300 + (x * 0x8))
#define MAC_ADDR_HI_AE                          BIT(31)

/*
 * MAC Timestamp Control
 * This register controls the operation of the System Time generator
 * and processing of PTP packets for timestamping in the Receiver
 */
#define MAC_TIMESTAMP_CTRL                      0x0B00
/* PTP Timestamp control register defines */
/* Timestamp Enable */
#define MAC_TIMESTAMP_CTRL_TSENA                BIT(0)
/* Timestamp Fine/Coarse Update */
#define MAC_TIMESTAMP_CTRL_TSCFUPDT             BIT(1)
/* Timestamp Initialize */
#define MAC_TIMESTAMP_CTRL_TSINIT               BIT(2)
/* Timestamp Update */
#define MAC_TIMESTAMP_CTRL_TSUPDT               BIT(3)
/* Timestamp Interrupt Trigger Enable */
#define MAC_TIMESTAMP_CTRL_TSTRIG               BIT(4)
/* Addend Reg Update */
#define MAC_TIMESTAMP_CTRL_TSADDREG             BIT(5)
/* Enable Timestamp for All Frames */
#define MAC_TIMESTAMP_CTRL_TSENALL              BIT(8)
/* Digital or Binary Rollover Control */
#define MAC_TIMESTAMP_CTRL_TSCTRLSSR            BIT(9)
/* Enable PTP packet Processing for Version 2 Format */
#define MAC_TIMESTAMP_CTRL_TSVER2ENA            BIT(10)
/* Enable Processing of PTP over Ethernet Frames */
#define MAC_TIMESTAMP_CTRL_TSIPENA              BIT(11)
/* Enable Processing of PTP Frames Sent over IPv6-UDP */
#define MAC_TIMESTAMP_CTRL_TSIPV6ENA            BIT(12)
/* Enable Processing of PTP Frames Sent over IPv4-UDP */
#define MAC_TIMESTAMP_CTRL_TSIPV4ENA            BIT(13)
/* Enable Timestamp Snapshot for Event Messages */
#define MAC_TIMESTAMP_CTRL_TSEVNTENA            BIT(14)
/* Enable Snapshot for Messages Relevant to Master */
#define MAC_TIMESTAMP_CTRL_TSMSTRENA            BIT(15)
/* Select PTP packets for Taking Snapshots */
#define MAC_TIMESTAMP_CTRL_SNAPTYPSEL_1         BIT(16)
/* Enable MAC address for PTP Frame Filtering */
#define MAC_TIMESTAMP_CTRL_TSENMACADDR          BIT(18)

/* Sub-Second Increment */
#define MAC_SUB_SECOND_INCR                     0x0B04
/* SSIR defines */
#define MAC_SUB_SECOND_INCR_SSINC_MASK          0xFF
#define MAC_SUB_SECOND_INCR_SSINC_SHIFT         16
/* PTP Clock for Fine Update in Hz*/
#ifdef CONFIG_BOARD_INTEL_PSE
#define PTP_CLOCK_FINE_UPDATE                   12500000
#else
#define PTP_CLOCK_FINE_UPDATE                   50000000
#endif

/* System Time – Seconds */
#define MAC_SYS_TIME_SEC                        0x0B08
/* System Time – Nanoseconds */
#define MAC_SYS_TIME_NANOSEC                    0x0B0C
/* System Time – Seconds Update */
#define MAC_SYS_TIME_SEC_UPD                    0x0B10
/* System Time – Nanoseconds Update */
#define MAC_SYS_TIME_NANOSEC_UPD                0x0B14
#define MAC_SYS_TIME_NANOSEC_UPD_ADDSUB         BIT(31)

/* Timestamp Addend Reg */
#define MAC_TIMESTAMP_ADDEND                    0x0B18

/* Timestamp Ingress & Egress Correction Registers */
#define MAC_TIMESTAMP_INGRESS_CORR_NANOSEC      0x0B58
#define MAC_TIMESTAMP_INGRESS_CORR_NEG_BIT      BIT(31)
#define MAC_TIMESTAMP_EGRESS_CORR_NANOSEC       0x0B5C

/* Timestamp ingress & egress latency Registers */
#define MAC_TIMESTAMP_INGRESS_LATENCY           0x0B68
#define MAC_TIMESTAMP_ITLNS_MASK                0x0FFF0000
#define MAC_TIMESTAMP_ITLNS_SHIFT               16
#define MAC_TIMESTAMP_EGRESS_LATENCY            0x0B6C
#define MAC_TIMESTAMP_ETLNS_MASK                0x0FFF0000
#define MAC_TIMESTAMP_ETLNS_SHIFT               16

/*
 * MTL Registers
 */
#define MTL_OPERATION_MODE                      0x0C00
#define MTL_OPR_MD_FRPE                         BIT(15)
#define INV_MTL_OPR_MD_SCHALG                   0xFFFFFF9F
#define MTL_OPR_MD_SCHALG_WRR                   0x00000000
#define MTL_OPR_MD_SCHALG_WFQ                   0x00000020
#define MTL_OPR_MD_SCHALG_DWRR                  0x00000040
#define MTL_OPR_MD_SCHALG_SP                    0x00000060
#define INV_MTL_OPR_MD_RAA_SP                   0xFFFFFFFB
#define MTL_OPR_MD_RAA_WSP                      BIT(2)

#define MTL_INTERRUPT_STATUS                    0x0C20
#define MTL_INTR_ESTIS                          BIT(18)
#define MTL_INTR_STS_Q7IS                       BIT(7)
#define MTL_INTR_STS_Q6IS                       BIT(6)
#define MTL_INTR_STS_Q5IS                       BIT(5)
#define MTL_INTR_STS_Q4IS                       BIT(4)
#define MTL_INTR_STS_Q3IS                       BIT(3)
#define MTL_INTR_STS_Q2IS                       BIT(2)
#define MTL_INTR_STS_Q1IS                       BIT(1)
#define MTL_INTR_STS_Q0IS                       BIT(0)
#define MTL_INTR_CTRL_STATUS_Q(x)               (0x0D2C + (x * 0x40))
#define MTL_QX_INTR_CTRL_STS_RXOIE              BIT(24)
#define MTL_QX_INTR_CTRL_STS_RXOVFIS            BIT(16)

#define MTL_RXQ_DMA_MAP0                        0x0C30
#define MTL_RXQ_DMA_MAP1                        0x0C34
#define INV_MTL_RXQ_DMA_Q0_Q4MDMACH             0xFFFFFFF8
#define INV_MTL_RXQ_DMA_Q1_Q5MDMACH             0xFFFFF8FF
#define INV_MTL_RXQ_DMA_Q2_Q6MDMACH             0xFFF8FFFF
#define INV_MTL_RXQ_DMA_Q3_Q7MDMACH             0xF8FFFFFF
#define MTL_RXQ_DMA_Q0_Q4MDMACH_0               0x00000000
#define MTL_RXQ_DMA_Q0_Q4MDMACH_1               0x00000001
#define MTL_RXQ_DMA_Q0_Q4MDMACH_2               0x00000002
#define MTL_RXQ_DMA_Q0_Q4MDMACH_3               0x00000003
#define MTL_RXQ_DMA_Q0_Q4MDMACH_4               0x00000004
#define MTL_RXQ_DMA_Q0_Q4MDMACH_5               0x00000005
#define MTL_RXQ_DMA_Q0_Q4MDMACH_6               0x00000006
#define MTL_RXQ_DMA_Q0_Q4MDMACH_7               0x00000007
#define MTL_RXQ_DMA_Q1_Q5MDMACH_0               0x00000000
#define MTL_RXQ_DMA_Q1_Q5MDMACH_1               0x00000100
#define MTL_RXQ_DMA_Q1_Q5MDMACH_2               0x00000200
#define MTL_RXQ_DMA_Q1_Q5MDMACH_3               0x00000300
#define MTL_RXQ_DMA_Q1_Q5MDMACH_4               0x00000400
#define MTL_RXQ_DMA_Q1_Q5MDMACH_5               0x00000500
#define MTL_RXQ_DMA_Q1_Q5MDMACH_6               0x00000600
#define MTL_RXQ_DMA_Q1_Q5MDMACH_7               0x00000700
#define MTL_RXQ_DMA_Q2_Q6MDMACH_0               0x00000000
#define MTL_RXQ_DMA_Q2_Q6MDMACH_1               0x00010000
#define MTL_RXQ_DMA_Q2_Q6MDMACH_2               0x00020000
#define MTL_RXQ_DMA_Q2_Q6MDMACH_3               0x00030000
#define MTL_RXQ_DMA_Q2_Q6MDMACH_4               0x00040000
#define MTL_RXQ_DMA_Q2_Q6MDMACH_5               0x00050000
#define MTL_RXQ_DMA_Q2_Q6MDMACH_6               0x00060000
#define MTL_RXQ_DMA_Q2_Q6MDMACH_7               0x00070000
#define MTL_RXQ_DMA_Q3_Q7MDMACH_0               0x00000000
#define MTL_RXQ_DMA_Q3_Q7MDMACH_1               0x01000000
#define MTL_RXQ_DMA_Q3_Q7MDMACH_2               0x02000000
#define MTL_RXQ_DMA_Q3_Q7MDMACH_3               0x03000000
#define MTL_RXQ_DMA_Q3_Q7MDMACH_4               0x04000000
#define MTL_RXQ_DMA_Q3_Q7MDMACH_5               0x05000000
#define MTL_RXQ_DMA_Q3_Q7MDMACH_6               0x06000000
#define MTL_RXQ_DMA_Q3_Q7MDMACH_7               0x07000000

#define MTL_TBS_CTRL                            0x0C40
#define MTL_TBS_CTRL_LEOS_MASK                  0xFFFFFF00
#define MTL_TBS_CTRL_LEOV                       BIT(1)
#define MTL_TBS_CTRL_ESTM                       BIT(0)

#define MTL_RXP_CTRL_STATS                      0x0CA0
#define MTL_RXP_CTRL_STATS_RXPI                 BIT(31)
#define MTL_RXP_INDRT_ACC_CTRL_STATS            0x0CB0
#define MTL_RXP_INDACC_CTRLSTS_STARTBUSY        BIT(31)
#define MTL_RXP_INDACC_CTRLSTS_WRRDN            BIT(16)
#define MTL_RXP_INDACC_CTRLSTS_ADDR_MASK        0x000003FF
#define MTL_RXP_INDRT_ACC_DATA                  0x0CB4
#define MTL_RXP_INSTR_ACCPT                     BIT(0)
#define MTL_RXP_INSTR_REJT                      BIT(1)
#define MTL_RXP_INSTR_NEXT                      BIT(3)
#define MTL_RXP_INSTR_FRM_OFF_UNIT              4
#define MTL_RXP_DWORD_PER_INSTR                 4

#define MTL_TXQ_OPERATION_MODE(x)               (0x0D00 + (x * 0x40))
#define MTL_TXQ_OPR_TQS_MASK                    0x007F0000
#define MTL_TXQ_OPR_TQS_SHIFT                   16
#define MTL_TXQSZ_BLOCK                         256
#define MTL_TXQ_OPR_TSF                         BIT(1)
#define INV_MTL_TXQ_OPR_TXQEN                   0xFFFFFFF3
#define MTL_TXQ_OPR_TXQEN_EN                    0x00000008
#define MTL_TXQ_OPR_TXQEN_EN_IF_AV              0x00000004

#define MTL_TXQ_DEBUG(x)                        (0x0D08 + (x * 0x40))
#define MTL_DEBUG_TXFSTS                        BIT(4)
#define MTL_DEBUG_TRCSTS_MASK                   0x00000006
#define MTL_DEBUG_TRCSTS_SHIFT                  1
#define MTL_DEBUG_TRCSTS_IDLE                   0
#define MTL_DEBUG_TRCSTS_READ                   1
#define MTL_DEBUG_TRCSTS_TXW                    2
#define MTL_DEBUG_TRCSTS_WRITE                  3

#define MTL_RXQ_OPERATION_MODE(x)               (0x0D30 + (x * 0x40))
#define MTL_RXQ_OPR_RQS_MASK                    0x07F00000
#define MTL_RXQ_OPR_RQS_SHIFT                   20
#define MTL_RXQSZ_BLOCK                         256
#define MTL_RXQ_OPR_RSF                         BIT(5)

/*
 * DMA Registers
 */
#define DMA_MODE                                0x1000
#define INV_DMA_MD_INTM                         0xFFFCFFFF
#define DMA_MD_INTM_MODE1                       BIT(16)
#define DMA_MD_INTM_MODE2                       BIT(17)
#define INV_DMA_MD_TAA                          0xFFFFFFE3
#define DMA_MD_TAA_FP                           0x00000000
#define DMA_MD_TAA_WSP                          0x00000004
#define DMA_MD_TAA_WRP                          0x00000008
#define DMA_MD_SWR                              BIT(0)

#define DMA_SYSBUS_MODE                         0x1004
#define DMA_SYSBUS_MD_EN_LPI                    BIT(31)
#define INV_DMA_SYSBUS_MD_WR_OSR_LMT            0xF8FFFFFF
#define INV_DMA_SYSBUS_MD_RD_OSR_LMT            0xFFF8FFFF
#define DMA_SYSBUS_MD_MB                        BIT(14)
#define DMA_SYSBUS_MD_AAL                       BIT(12)
#define DMA_SYSBUS_MD_AALE                      BIT(10)
#define DMA_SYSBUS_MD_BLEN32                    BIT(4)
#define DMA_SYSBUS_MD_BLEN16                    BIT(3)
#define DMA_SYSBUS_MD_BLEN8                     BIT(2)
#define DMA_SYSBUS_MD_BLEN4                     BIT(1)
#define DMA_SYSBUS_MD_FB                        BIT(0)

#define DMA_INTERRUPT_STATUS                    0x1008
#define DMA_INTR_STS_CHNL_MAX                   8
#define DMA_INTR_STS_DCIS(x)                    (BIT(0) << x)

#define DMA_TBS_CTRL                            0x1050
#define DMA_TBS_CTRL_FTOS_MASK                  0xFFFFFF00
#define DMA_TBS_CTRL_FTOV                       BIT(0)

#define DMA_CONTROL_CH(x)                       (0x1100 + (x * 0x80))
#define DMA_CH_CTRL_DSL_MASK                    0x001C0000
#define DMA_CH_CTRL_DSL_SHIFT                   18
#define DMA_CH_CTRL_PBLX8                       BIT(16)
#define DMA_CH_CTRL_MSS_MASK                    0x00003FFF

#define DMA_TX_CONTROL_CH(x)                    (0x1104 + (x * 0x80))
#define DMA_CH_TX_CTRL_EDSE                     BIT(28)
#define DMA_CH_TX_CTRL_TXPBL_MASK               0x003F0000
#define DMA_CH_TX_CTRL_TXPBL_SHIFT              16
#define DMA_CH_TX_CTRL_OSF                      BIT(4)
#define DMA_CH_TX_CTRL_ST                       BIT(0)

#define DMA_RX_CONTROL_CH(x)                    (0x1108 + (x * 0x80))
#define DMA_CH_RX_CTRL_RXPBL_MASK               0x003F0000
#define DMA_CH_RX_CTRL_RXPBL_SHIFT              16
#define DMA_CH_RX_CTRL_RBSZ_MASK                0x00007FFE
#define DMA_CH_RX_CTRL_RBSZ_SHIFT               1
#define DMA_CH_RX_CTRL_SR                       BIT(0)

#define DMA_TXDESC_LIST_ADDR_CH(x)              (0x1114 + (x * 0x80))
#define DMA_RXDESC_LIST_ADDR_CH(x)              (0x111C + (x * 0x80))
#define DMA_TXDESC_TAIL_PTR_CH(x)               (0x1120 + (x * 0x80))
#define DMA_RXDESC_TAIL_PTR_CH(x)               (0x1128 + (x * 0x80))
#define DMA_TXDESC_RING_LENGTH_CH(x)            (0x112C + (x * 0x80))
#define DMA_RXDESC_RING_LENGTH_CH(x)            (0x1130 + (x * 0x80))

#define DMA_INTR_EN_CH(x)                       (0x1134 + (x * 0x80))
#define DMA_CH_INTR_EN_NIE                      BIT(15)
#define DMA_CH_INTR_EN_RWTE                     BIT(9)
#define DMA_CH_INTR_EN_RIE                      BIT(6)
#define DMA_CH_INTR_EN_TIE                      BIT(0)
#define DMA_INTR_STATUS_CH(x)                   (0x1160 + (x * 0x80))
#define DMA_CH_INTR_STS_NIS                     BIT(15)
#define DMA_CH_INTR_STS_RWT                     BIT(9)
#define DMA_CH_INTR_STS_RI                      BIT(6)
#define DMA_CH_INTR_STS_TI                      BIT(0)

#define DMA_RX_INTR_WDT_CH(x)                   (0x1138 + (x * 0x80))
#define DMA_CH_RX_INTR_WDT_RWTU_MASK            0x00030000
#define DMA_CH_RX_INTR_WDT_RWTU_SHIFT           16
#define DMA_CH_RX_INTR_WDT_RWTU(x)              (256 * POW2(x))
#define DMA_CH_RX_INTR_WDT_RWT_MASK             0x000000FF

/*
 * XPCS Registers
 */
#define XPCS_ADDR                               0x16
#define VENDOR_SPECIFIC_CTRL_MMD                0x1E
#define VENDOR_SPECIFIC_MII_MMD                 0x1F
#define SR_MII_MMD_CTRL_REG                     0x0000
#define SR_MII_CTRL_RST                         BIT(15)
#define SR_MII_CTRL_AN_EN                       BIT(12)
#define SR_MII_CTRL_RESTART_AN                  BIT(9)
#define SR_MII_DEV_ID_1_REG                     0x0002
#define SR_MII_DEV_ID_2_REG                     0x0003
#define SR_MII_TIME_SYNC_ABL                    0x0708
#define SR_MII_TIME_SYNC_MII_TX_DLY_ABL         BIT(1)
#define SR_MII_TIME_SYNC_MII_RX_DLY_ABL         BIT(0)
#define SR_MII_TIME_SYNC_TX_MAX_DLY_LWR         0x0709
#define SR_MII_TIME_SYNC_TX_MAX_DLY_UPR         0x070A
#define SR_MII_TIME_SYNC_TX_MIN_DLY_LWR         0x070B
#define SR_MII_TIME_SYNC_TX_MIN_DLY_UPR         0x070C
#define SR_MII_TIME_SYNC_RX_MAX_DLY_LWR         0x070D
#define SR_MII_TIME_SYNC_RX_MAX_DLY_UPR         0x070E
#define SR_MII_TIME_SYNC_RX_MIN_DLY_LWR         0x070F
#define SR_MII_TIME_SYNC_RX_MIN_DLY_UPR         0x0710
#define SR_MII_TIME_SYNC_DLY_UPR_SHIFT          16
#define VR_MII_MMD_DIG_CTRL1_REG                0x8000
#define VR_MII_DIG_CTRL1_MAC_AUTO_SW            BIT(9)
#define VR_MII_DIG_CTRL1_PRE_EMP                BIT(6)
#define VR_MII_DIG_CTRL1_EN_2_5G_MD             BIT(2)
#define VR_MII_DIG_CTRL1_PHY_MODE_CTRL          BIT(0)
#define VR_MII_MMD_AN_CTRL_REG                  0x8001
#define VR_MII_AN_CTRL_TX_CFG                   BIT(3)
#define VR_MII_AN_CTRL_TX_CFG_PHY_SIDE_SGMII    0x00000008
#define VR_MII_AN_CTRL_TX_CFG_MAC_SIDE_SGMII    0x00000000
#define VR_MII_AN_CTRL_PCS_MODE_MASK            0x00000006
#define VR_MII_AN_CTRL_PCS_MODE_SGMII           0x00000004
#define VR_MII_AN_CTRL_PCS_MODE_QSGMII          0x00000006
#define VR_MII_AN_CTRL_AN_INTR_EN               BIT(0)
#define VR_MII_EEE_MCTRL0                       0x8006
#define VR_MII_EEE_LTX_EN                       BIT(0)
#define VR_MII_EEE_LRX_EN                       BIT(1)
#define VR_MII_EEE_TX_QUIET_EN                  BIT(2)
#define VR_MII_EEE_RX_QUIET_EN                  BIT(3)
#define VR_MII_EEE_TX_EN_CTRL                   BIT(4)
#define VR_MII_EEE_RX_EN_CTRL                   BIT(7)
#define VR_MII_EEE_MCTRL1                       0x800B
#define VR_MII_EEE_TRN_LPI                      BIT(0)

/*
 * ModPhy Registers
 */
#define MODPHY_ADDR                             0x15
#define MODPHY_GPCR0                            0x0B
#define MODPHY_GPCR0_PCLK_RATE_MASK             0x00007000
#define MODPHY_GPCR0_PCLK_RATE_1G               1
#define MODPHY_GPCR0_PCLK_RATE_2_5G             0
#define MODPHY_GPCR0_PCLK_RATE_SHIFT            12
#define MODPHY_GPCR0_PWRDWN_SUS_MASK            0x00000070
#define MODPHY_GPCR0_PWRDWN_SUS_P0              0
#define MODPHY_GPCR0_PWRDWN_SUS_P3              3
#define MODPHY_GPCR0_PWRDWN_SUS_SHIFT           4
#define MODPHY_GPCR0_RSTDATAPATH_INTF           BIT(2)
#define MODPHY_GPCR0_PHYRXCLK_REQ               BIT(1)
#define MODPHY_GPCR0_PLLREFCLK_REQ              BIT(0)
#define MODPHY_GPSR0                            0x05
#define MODPHY_GPSR0_PWRDWN_SUS_SIG_MASK        0x00000070
#define MODPHY_GPSR0_PWRDWN_SUS_SIG_SHIFT       4
#define MODPHY_GPSR0_PWRDWN_SUS_SIG_P0          0
#define MODPHY_GPSR0_PWRDWN_SUS_SIG_P3          3
#define MODPHY_GPSR0_PLLREFCLK_VALID            BIT(0)
#define MODPHY_GPSR0_PLLREFCLK_SHIFT            0

/*
 * Statistics Registers
 */
#define TX_BROADCAST_PACKETS_GOOD               0x71C
#define TX_MULTICAST_PACKETS_GOOD               0x720
#define TX_OCTET_COUNT_GOOD                     0x764
#define TX_PACKET_COUNT_GOOD                    0x768
#define RX_OCTET_COUNT_GOOD                     0x788
#define RX_BROADCAST_PACKETS_GOOD               0x78C
#define RX_MULTICAST_PACKETS_GOOD               0x790
#define RX_UNICAST_PACKETS_GOOD                 0x7C4
#define RX_CONTROL_PACKETS_GOOD                 0x7E4

/* Network Proxy mode register */
#define MISC_PROXYMODE_REG_GBE0                 0x330
#define MISC_PROXYMODE_REG_GBE1                 0x334
#define MISC_GBE_PROXYMODE_BIT                  BIT(0)

/* Proxy mode entry interrupt register. */
#define MISC_PROXYMODE_ENTRY_INT_STS            0x338

/* SRAM Region 1 for address protection */
#define MISC_SRAM_REGION_1_BASE			0x400
#define MISC_SRAM_REGION_1_SIZE			0x404
#define MISC_SRAM_REGION_1_SIZE_128K		0x40

/* TODO: change to sedi API.
 * ATT register for Address Translation Table.
 */
#define ATT_MMIO_BASE_REG_ENTRY_6               0x60
#define ATT_MMIO_BASE_REG_ENTRY_8               0x80
#define ATT_MMIO_BASE_VALID                     BIT(0)

/*
 * ATT Register for OCP Translate Offset
 */
#define ATT_OCP_XLATE_OFFSET_REG_ENTRY_6        0x68
#define ATT_OCP_XLATE_OFFSET_REG_ENTRY_8        0x88
#define ATT_OCP_XLATE_OFFSET_MASK               0xFFFFF000

/* Shared memory for A2H Packets */
#define NETPROX_MMIO_GBE_L2                    0x600E0000
#define NETPROX_MMIO_GBE_L2_SIZE               (128 * 1024)

#define MISC_DMA_CONTROL_CH(x)                  (0x10000 + (x * 0x4))
#define MISC_DMA_CONTROL_TRANSFER_MODE_MASK     0x00000003
#define MISC_DMA_CONTROL_CH_MAX                 15

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_ETHERNET_ETH_DWC_EQOS_PRIV_H_ */
