/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_ETHERNET_ETH_DWC_EQOS_NETPROX_H_
#define DRIVERS_ETHERNET_ETH_DWC_EQOS_NETPROX_H_

#include "network_proxy_common.h"

struct netprox_net {
	struct net_if *iface;
	int (*np_exit)(struct net_if *iface);
	int (*arp_offload)(struct net_if *iface, bool enable, uint32_t ip_addr);
	void (*np_eth_tx_data)(struct net_if *iface, uint8_t *data,
			       uint16_t size);
	uint32_t np_vnn_id;
	bool np_cls_in_process;
	bool np_phy_irq_in_process;
	bool np_rx_data_in_process[CONFIG_ETH_DWC_EQOS_RX_QUEUES];
};

struct netprox_a2h_rxbuf {
	/* Refer to A2H Memory Pool Header Format definition
	 * in network_proxy_common.h.
	 */
	uint16_t a2h_total_packets;
	uint16_t a2h_total_size;
	/* A2H memory pool addr */
	uint32_t a2h_addr;
	/* A2H  memory pool max size */
	uint32_t a2h_max;
};

struct netprox_shmem_info {
	/* Shared memory pool addr */
	uint32_t np_shmem_addr;
	/* Shared memory pool size */
	uint32_t np_shmem_sz;
};

struct netprox_context {
	/* Ethernet driver context */
	struct netprox_net *net_ctx;
	/* Agent to Host receive buffer */
	struct netprox_a2h_rxbuf a2h_rxbuf;
	/* Shared memory information */
	struct netprox_shmem_info shmem_info;
	/* When host has waken-up, the agent will no longer performance frame
	 * classification. This flag is used to mark that all frames received
	 * by frame classifier is automatically added to A2H Rx buffer pool.
	 */
	bool add_a2h_pkt;
	/* already wake host */
	bool waked_host;
};

#define NETPROX_EXIT_POLLING_INTERVAL			K_MSEC(100)
#define NETPROX_EXIT_POLLING_COUNT_MAX			5

void netprox_register(struct netprox_net *net_ctx);
void netprox_register_a2h_info(uint32_t a2h_addr, uint32_t a2h_max);
void netprox_register_shmem_info(uint32_t np_shmem_addr, uint32_t np_shmem_sz);

void frm_cls_add_pkt(uint8_t *rxbuf, uint16_t len);
void netprox_add_a2h_pkt_wake(uint8_t *data, uint16_t len);
void netprox_configure_arp_offload(void);

void netprox_get_ipv4_addr(uint32_t *ipv4_addr);
void netprox_get_mac_addr(struct net_eth_addr *mac_addr);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_ETHERNET_ETH_DWC_EQOS_NETPROX_H_ */
