/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define LOG_MODULE_NAME eth_dwc_eqos_netprox
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <kernel.h>
#include <init.h>
#include <string.h>
#include <net/ethernet.h>
#include <net/net_ip.h>

#include "icmpv4.h"
#include "icmpv6.h"
#include "tcp_internal.h"
#include "heci.h"
#include "eth_dwc_eqos_v5_netprox_proto.h"
#include "sedi.h"

/* Define global variable to store incoming packets for Frame Classifier */
static struct frame_classifier frm_cls;

/* Define global variable to store Network Proxy Management Information Base */
static struct np_mib np_mib = { 0 };

/* Define global variable to store SNMP OID tree information */
static struct snmp_oid_tree_info snmp_oid_tree_info = { 0 };

/* Define global variable to store mDNS Resource Record database */
static struct mdns_db mdns_db = { 0 };
static struct mdns_db_hdr *mdns_db_hdr =
	(struct mdns_db_hdr *)mdns_db.data;

/* Define handling decision for IPv4, IPv6, tcp, SNMP, and mDNS protocols */
uint16_t fc_rs_ipv4, fc_rs_ipv6, fc_rs_tcp_wake_port, fc_rs_snmp, fc_rs_mdns;
/* FC_RS_SIZE highly depends on data type use for above fc_rs_xxx */
#define FC_RS_SIZE                      sizeof(uint16_t)

/* Define global variable for IPC message */
static heci_rx_msg_t np_ipc_rx_msg;
static uint8_t np_ipc_rx_buf[NP_IPC_MSG_MAX];
static uint8_t np_ipc_tx_buf[NP_IPC_MSG_MAX];

static K_THREAD_STACK_DEFINE(netprox_stack, NETPROX_STACK_SIZE);
static struct k_thread netprox_thread;

static K_THREAD_STACK_DEFINE(netprox_cls_wq_stack, NETPROX_CLS_WQ_STACK_SIZE);

static K_SEM_DEFINE(netprox_event_sem, 0, 1);
static uint32_t netprox_event;
static uint32_t netprox_conn_id;

struct netprox_context np_ctx = { .net_ctx = NULL,
				  .a2h_rxbuf.a2h_total_packets = 0,
				  .a2h_rxbuf.a2h_total_size = A2H_HEADER_SIZE,
				  .a2h_rxbuf.a2h_addr = 0,
				  .a2h_rxbuf.a2h_max = 0,
				  .shmem_info.np_shmem_addr = 0,
				  .shmem_info.np_shmem_sz = 0,
				  .add_a2h_pkt = false,
				  .waked_host = false };

K_MUTEX_DEFINE(shmem_mutex);

static inline void dcache_invalidate(uint32_t addr, uint32_t size)
{
	/* Align address to 32 bytes */
	uint32_t start_addr = addr & (uint32_t) ~(NETPROX_DCACHE_ALIGNMENT - 1);
	uint32_t size_full = size + addr - start_addr;

	sedi_core_inv_dcache_by_addr((uint32_t *)start_addr, size_full);
}

static inline void dcache_clean(uint32_t addr, uint32_t size)
{
	/* Align address to 32 bytes */
	uint32_t start_addr = addr & (uint32_t) ~(NETPROX_DCACHE_ALIGNMENT - 1);
	uint32_t size_full = size + addr - start_addr;

	sedi_core_clean_dcache_by_addr((uint32_t *)start_addr, size_full);
}

/* Add Agent-to-Host packet into shared memory pool */
void netprox_add_a2h_packet(uint8_t *data, uint16_t len)
{
	int size;
	int pkt_offset;
	void *ptr;
	mem_addr_t a2h_addr;
	struct np_a2h_pool_header pool_hdr = { 0 };
	struct np_a2h_packet_header pkt_hdr = { 0 };

	k_mutex_lock(&shmem_mutex, K_FOREVER);
	size = np_ctx.a2h_rxbuf.a2h_total_size + A2H_NEW_PKT_SIZE;
	/* A2H WQ full, drop packet */
	if (size > np_ctx.a2h_rxbuf.a2h_max) {
		LOG_DBG("A2H packets are dropped due to buffer full.");
	} else {
		/* Write packet header */
		pkt_hdr.pkt_len = len;
		/* Offset to packet n */
		pkt_offset = (np_ctx.a2h_rxbuf.a2h_total_packets *
			      A2H_NEW_PKT_SIZE);
		a2h_addr = ((np_ctx.a2h_rxbuf.a2h_addr + A2H_HEADER_SIZE) +
			    pkt_offset);

		/* memcpy packet header */
		ptr = (void *)a2h_addr;
		memcpy(ptr, (void *)&pkt_hdr, A2H_PKT_HEADER_SIZE);
		dcache_clean((uint32_t)ptr, A2H_PKT_HEADER_SIZE);

		/* memcpy packet content */
		a2h_addr += A2H_PKT_HEADER_SIZE;
		ptr = (void *)a2h_addr;
		memcpy(ptr, data, len);
		dcache_clean((uint32_t)ptr, len);

		np_ctx.a2h_rxbuf.a2h_total_size += A2H_NEW_PKT_SIZE;
		np_ctx.a2h_rxbuf.a2h_total_packets++;
	}

	/* Write A2H Memory Pool Header Format */
	pool_hdr.total_packets = np_ctx.a2h_rxbuf.a2h_total_packets;
	sys_write16(pool_hdr.total_packets, np_ctx.a2h_rxbuf.a2h_addr);
	dcache_clean((uint32_t)np_ctx.a2h_rxbuf.a2h_addr,
		     sizeof(pool_hdr.total_packets));

	pool_hdr.total_size = np_ctx.a2h_rxbuf.a2h_total_size;
	sys_write16(pool_hdr.total_size, (np_ctx.a2h_rxbuf.a2h_addr + 2));
	dcache_clean((uint32_t)(np_ctx.a2h_rxbuf.a2h_addr + 2),
		     sizeof(pool_hdr.total_size));

	k_mutex_unlock(&shmem_mutex);
}

static void netprox_send_message(uint32_t command, uint8_t *buf, uint32_t size)
{
	struct np_ipc_hdr *txipc_hdr;
	mrd_t m = { 0 };

	txipc_hdr = (struct np_ipc_hdr *)np_ipc_tx_buf;
	txipc_hdr->command = command;
	txipc_hdr->status = 0;
	txipc_hdr->size = size;

	if (size > NP_IPC_PYLD_MAX) {
		LOG_ERR("IPC message size (%d bytes) bigger than supported "
			"payload (%d bytes).", size, NP_IPC_PYLD_MAX);
		return;
	}

	if (size > 0) {
		memcpy(txipc_hdr + 1, buf, size);
	}

	m.buf = np_ipc_tx_buf;
	m.len = sizeof(*txipc_hdr) + size;
	heci_send(netprox_conn_id, &m);
}

static void netprox_wake_host(void)
{
	if (!np_ctx.waked_host) {
		np_ctx.waked_host = true;

		/* Assert VNN_REQ and wait for VNN_ACK */
		sedi_pm_vnn_request(np_ctx.net_ctx->np_vnn_id, 1);

		netprox_send_message(NP_A2H_CMD_HOST_IS_AWAKE, NULL, 0);
	}
}

void netprox_add_a2h_pkt_wake(uint8_t *data, uint16_t len)
{
	netprox_add_a2h_packet(data, len);
	netprox_wake_host();
}

/* From subsys/net/ip/utils.c */
static uint16_t netprox_calc_chksum(uint16_t sum, const uint8_t *ptr,
				    uint16_t len)
{
	uint16_t tmp;
	const uint8_t *end;

	end = ptr + len - 1;

	while (ptr < end) {
		tmp = (ptr[0] << 8) + ptr[1];
		sum += tmp;
		if (sum < tmp) {
			sum++;
		}
		ptr += 2;
	}

	if (ptr == end) {
		tmp = ptr[0] << 8;
		sum += tmp;
		if (sum < tmp) {
			sum++;
		}
	}

	return sum;
}

/* Frame Classifier: Process TCP packet */
static void fc_process_tcp(struct frame_classifier *frame,
			   struct net_tcp_hdr *tcp_hdr)
{
	int i;
	uint8_t fc_tcp_decision;

	if (!((fc_rs_tcp_wake_port & NP_RL_CLS_ENABLE) == NP_RL_CLS_ENABLE)) {
		LOG_DBG("Ignoring TCP packet: handling decision = disable.");
		return;
	}

	fc_tcp_decision = (fc_rs_tcp_wake_port & ~NP_RL_CLS_ENABLE);

	for (i = 0; i < np_mib.tcp_port_sz; i++) {
		if (htons(np_mib.tcp_port[i]) == tcp_hdr->dst_port) {
			if (fc_tcp_decision == NP_RL_CLS_DROP) {
				LOG_DBG("Drop TCP pkt: Port:%d.",
					np_mib.tcp_port[i]);
				return;
			} else if (fc_tcp_decision == NP_RL_CLS_WAKE) {
				if (tcp_hdr->flags == SYN) {
					LOG_DBG("Wake Host to pass TCP SYNC "
						"pkt: Port:%d.",
						np_mib.tcp_port[i]);
					netprox_add_a2h_packet(frame->pkt,
							       frame->len);
					netprox_wake_host();
					return;
				}
			}
		}
	}

	LOG_DBG("Ignoring TCP packet: not matching destination port / flags.");
}

/* Fill in the length of long form data (length > 127) according to following
 * format:
 * 1. The initial octet shall be encoded as follows:
 *    a) bits 0 - 6 shall encode the number of subsequent octets
 *    b) bit 7 shall be one
 * 2. The subsequent octets should record the length of value
 */
static void fr_snmp_fill_len(uint8_t *snmp_hdr, uint8_t field_offset,
			     uint16_t len)
{
	SNMP_FIELD_LEN(field_offset) = NETPROX_SNMP_LONG_FORM_TWO_OCTET;
	SNMP_FIELD_LEN_FIRST_OCTET(field_offset) =
		(uint8_t)SNMP_GET_UPPER_BYTE_LEN(len);
	SNMP_FIELD_LEN_SECOND_OCTET(field_offset) = (uint8_t)len;
}

/* Adjust the offset of SNMP PDU, Varbind List, Varbind, and Object Identifier
 * fields
 */
static void fr_snmp_adjust_offset(struct snmp_msg_field_offset *field_offset,
				  uint8_t *snmp_hdr)
{
	int i;

	/* Shift Request ID, Error, and Error Index */
	for (i = field_offset->vbl; i >= field_offset->id; i--) {
		SNMP_FIELD(i + NETPROX_SNMP_FOUR_SUB_OCTET) = SNMP_FIELD(i);
	}
	field_offset->vbl += NETPROX_SNMP_FOUR_SUB_OCTET;
	field_offset->vb += NETPROX_SNMP_SIX_SUB_OCTET;
	field_offset->oid += NETPROX_SNMP_EIGHT_SUB_OCTET;

	/* Shift SNMP Version and SNMP Community String */
	for (i = field_offset->pdu; i >= field_offset->ver; i--) {
		SNMP_FIELD(i + NETPROX_SNMP_TWO_SUB_OCTET) = SNMP_FIELD(i);
	}
	field_offset->pdu += NETPROX_SNMP_TWO_SUB_OCTET;
}

/* Check offset to struct oid_node is within the range of OID tree */
static inline int fr_snmp_check_range_node(uint32_t offset)
{
	if (offset + sizeof(struct oid_node) >
	    snmp_oid_tree_info.oid_tree_sz) {
		LOG_DBG("Offset to OID tree (%d) is out of range.",
			offset);
		return -1;
	}

	return 0;
}

/* Check offset to struct oid_data_info is within the range of OID tree */
static inline int fr_snmp_check_range_data_info(uint32_t offset)
{
	if (offset + sizeof(struct oid_data_info) >
	    snmp_oid_tree_info.oid_tree_sz) {
		LOG_DBG("Offset to OID tree (%d) is out of range.",
			offset);
		return -1;
	}

	return 0;
}

/* Check offset to uint8_t OID data is within the range of OID tree */
static inline int fr_snmp_check_range_data(uint32_t offset, uint32_t size)
{
	if (offset + size > snmp_oid_tree_info.oid_tree_sz) {
		LOG_DBG("Offset to OID tree (%d) is out of range.",
			offset);
		return -1;
	}

	return 0;
}

/* Retrieve requested OID from SNMP OID tree */
static int fr_snmp_retrieve_oid(uint8_t depth,
				uint8_t oid[NETPROX_SNMP_MAX_OID_DEPTH],
				bool get_next, struct oid_node **node)
{
	int found;
	int ret = 0;

	*node = SNMP_OID_NODE_PTR(0);

	/* Search OID node */
	for (int i = 0; i < depth; i++) {
		if ((*node)->id != oid[i]) {
			found = 0;
			while ((*node)->next != 0) {
				if (fr_snmp_check_range_node((*node)->next)) {
					ret = -1;
					goto error;
				}
				*node = SNMP_OID_NODE_PTR((*node)->next);
				if ((*node)->id == oid[i]) {
					found = 1;
					if (i == depth - 1) {
						break;
					}
					if (fr_snmp_check_range_node(
						    (*node)->child)) {
						ret = -1;
						goto error;
					}
					*node = SNMP_OID_NODE_PTR(
						(*node)->child);
					break;
				}
			}
			if (!found) {
				ret = -1;
				goto error;
			}
		} else {
			if (i != depth - 1) {
				if (fr_snmp_check_range_node((*node)->child)) {
					ret = -1;
					goto error;
				}
				*node = SNMP_OID_NODE_PTR((*node)->child);
			}
		}
	}

	/* If the node does not has oid_data_info and is not GetNextRequest,
	 * then it is incorrect.
	 */
	if (!(*node)->has_info && !get_next) {
		ret = -1;
		goto error;
	}

error:
	return ret;
}

/* Retrieve the next OID from SNMP OID tree */
static int fr_snmp_retrieve_next_oid(uint8_t depth, struct oid_node **node)
{
	int found = 0;
	int ret = 0;

	/* If current requested OID is not the leaf node, continue to trace
	 * down the branch until the leaf node. Leaf node is the node that is
	 * the most bottom OID node that has oid_data_info (OID value).
	 */
	if (!(*node)->has_info) {
		while (!(*node)->has_info) {
			if (fr_snmp_check_range_node((*node)->child)) {
				ret = -1;
				break;
			}
			*node = SNMP_OID_NODE_PTR((*node)->child);
		}
		goto done;
	}

	/* If current requested OID is the leaf node (contains oid_data_info),
	 * then we get the next node at the same level as the requested OID.
	 * If we fail t0 do so, we continue to get the next node that is one
	 * or more levels higher relative to current requested OID.
	 */
	for (int i = 0; i < depth - 1; i++) {
		if ((*node)->next) {
			if (fr_snmp_check_range_node((*node)->next)) {
				break;
			}
			*node = SNMP_OID_NODE_PTR((*node)->next);
			found = 1;
			break;
		}
		if (fr_snmp_check_range_node((*node)->parent)) {
			break;
		}
		*node = SNMP_OID_NODE_PTR((*node)->parent);
	}
	if (!found) {
		ret = -1;
		goto done;
	}

	/* Continue to trace down the branch until the leaf node */
	while (!(*node)->has_info) {
		if (fr_snmp_check_range_node((*node)->child)) {
			ret = -1;
			break;
		}
		*node = SNMP_OID_NODE_PTR((*node)->child);
	}

done:
	return ret;
}

/* Get the full OID from leaf node */
static int fr_snmp_get_oid_id(uint8_t depth, struct oid_node *last_node,
			      uint8_t oid[NETPROX_SNMP_MAX_OID_DEPTH])
{
	uint32_t offset;
	struct oid_node *node;
	int ret = 0;

	oid[depth - 1] = last_node->id;
	offset = last_node->parent;

	/* Search parent */
	for (int i = 0; i < depth - 1; i++) {
		if (fr_snmp_check_range_node(offset)) {
			ret = -1;
			break;
		}
		node = SNMP_OID_NODE_PTR(offset);
		oid[depth - 2 - i] = node->id;
		offset = node->parent;
	}

	return ret;
}

/* Frame Responder: Process SNMP response message */
static void fr_process_snmp(struct frame_classifier *frame,
			    struct snmp_response_info *response_info,
			    struct snmp_msg_field_offset *msg_field_offset)
{
	struct oid_node *oid_node = NULL;
	struct oid_data_info *oid_data_info = NULL;
	struct net_eth_hdr *eth_hdr;
	struct net_ipv4_hdr *ip4_hdr = NULL;
	struct net_ipv6_hdr *ip6_hdr = NULL;
	struct net_udp_hdr *udp_hdr;
	struct in_addr host_addr4;
	struct in6_addr host_addr6;
	uint8_t *snmp_hdr;
	int pkt_len;
	uint16_t add_len;

	/* Retrieve requested OID */
	if (fr_snmp_retrieve_oid(response_info->oid_total_depth,
				 response_info->oid, response_info->get_next,
				 &oid_node)) {
		goto nosuchname;
	}

	/* Retrieve requested next OID */
	if (response_info->get_next) {
		if (fr_snmp_retrieve_next_oid(response_info->oid_total_depth,
					      &oid_node)) {
			goto nosuchname;
		}
		response_info->oid_total_depth = oid_node->depth;
		/* Update response_info->oid with next OID. */
		if (fr_snmp_get_oid_id(response_info->oid_total_depth, oid_node,
				       response_info->oid)) {
			goto nosuchname;
		}
	}

	if (fr_snmp_check_range_data_info(oid_node->child)) {
		goto nosuchname;
	}

	/* Obtain OID type, length, and value */
	oid_data_info = SNMP_OID_DATA_INFO_PTR(oid_node->child);
	response_info->val_type = oid_data_info->data_type;
	response_info->val_len = oid_data_info->data_len;
	if (response_info->val_len <= SNMP_OID_DATA_SZ) {
		response_info->val = (uint8_t *)&oid_data_info->data;
	} else {
		if (fr_snmp_check_range_data(oid_data_info->data,
					     response_info->val_len)) {
			goto nosuchname;
		}
		response_info->val = SNMP_OID_DATA_PTR(oid_data_info->data);
	}

	oid_data_info->request_count += 1;
	goto respond;

nosuchname:
	if (fc_rs_snmp & NP_RL_CLS_SUPP_SNMP_GALL) {
		LOG_DBG("Wake Host to pass SNMP message with unknown OID.");
		netprox_add_a2h_pkt_wake(frame->pkt, frame->len);
		return;
	}
	/* val_len = 0 is used to indicate NoSuchName SNMP Error. */
	response_info->val_len = 0;

respond:
	/* Get header */
	eth_hdr = (struct net_eth_hdr *)frame->pkt;
	if (response_info->is_ipv6) {
		ip6_hdr = (struct net_ipv6_hdr *)(eth_hdr + 1);
		udp_hdr = (struct net_udp_hdr *)(ip6_hdr + 1);
	} else {
		ip4_hdr = (struct net_ipv4_hdr *)(eth_hdr + 1);
		udp_hdr = (struct net_udp_hdr *)(ip4_hdr + 1);
		ip4_hdr->chksum = 0;
	}
	udp_hdr->chksum = 0;
	snmp_hdr = (uint8_t *)(udp_hdr + 1);

	/* Swap src and dst mac address */
	memcpy(&eth_hdr->dst, &eth_hdr->src, NP_MAC_ADDR_BYTES);
	memcpy(&eth_hdr->src, np_mib.mac_addr, NP_MAC_ADDR_BYTES);

	/* Swap src and dst ip address */
	if (response_info->is_ipv6) {
		net_ipaddr_copy(&host_addr6, &ip6_hdr->dst);
		net_ipaddr_copy(&ip6_hdr->dst, &ip6_hdr->src);
		net_ipaddr_copy(&ip6_hdr->src, &host_addr6);
	} else {
		net_ipaddr_copy(&host_addr4, &ip4_hdr->dst);
		net_ipaddr_copy(&ip4_hdr->dst, &ip4_hdr->src);
		net_ipaddr_copy(&ip4_hdr->src, &host_addr4);
	}

	/* Swap src and dst UDP port */
	memcpy(&udp_hdr->dst_port, &udp_hdr->src_port, NETPROX_UDP_PORT_BYTES);
	udp_hdr->src_port = htons(NETPROX_SNMP_DEFAULT_UDP_PORT);

	/* Set type of PDU field to GetResponse */
	SNMP_FIELD_TYPE(msg_field_offset->pdu) = NETPROX_SNMP_PDU_RESPONSE;

	if (response_info->val_len) {
		/* Calculate the additional SNMP response message length
		 * based on:-
		 *  1) Increase in OID depth (for GetNext)
		 *  2) The length of Value field (for GetResponse)
		 *  3) Two additional subsequent length octets for Value field
		 */
		add_len = response_info->val_len +
			  response_info->oid_total_depth -
			  SNMP_FIELD_LEN(msg_field_offset->oid) +
			  NETPROX_SNMP_TWO_SUB_OCTET;

		/* Long form data format is used, where the length octets shall
		 * consist of an initial octet and two subsequent octets.
		 */
		response_info->vb_len = SNMP_FIELD_LEN(msg_field_offset->vb) +
					add_len;
		add_len += NETPROX_SNMP_TWO_SUB_OCTET;
		response_info->vbl_len = SNMP_FIELD_LEN(msg_field_offset->vbl) +
					 add_len;
		add_len += NETPROX_SNMP_TWO_SUB_OCTET;
		response_info->pdu_len = SNMP_FIELD_LEN(msg_field_offset->pdu) +
					 add_len;
		add_len += NETPROX_SNMP_TWO_SUB_OCTET;
		response_info->msg_len = SNMP_FIELD_LEN(0) + add_len;
		add_len += NETPROX_SNMP_TWO_SUB_OCTET;

		/* Adjust the offset of SNMP PDU, Varbind List, Varbind, and
		 * Object Identifier fields
		 */
		fr_snmp_adjust_offset(msg_field_offset, snmp_hdr);

		/* Fill in the length of SNMP Message, SNMP PDU, Varbind List,
		 * and Varbind fields
		 */
		fr_snmp_fill_len(snmp_hdr, 0, response_info->msg_len);
		fr_snmp_fill_len(snmp_hdr, msg_field_offset->pdu,
				 response_info->pdu_len);
		fr_snmp_fill_len(snmp_hdr, msg_field_offset->vbl,
				 response_info->vbl_len);
		fr_snmp_fill_len(snmp_hdr, msg_field_offset->vb,
				 response_info->vb_len);

		/* Fill in the type of Varbind field */
		SNMP_FIELD_TYPE(msg_field_offset->vb) =
			NETPROX_SNMP_TYPE_SEQUENCE;

		/* Fill in the type, length, and value of Object Identifier
		 * field
		 */
		SNMP_FIELD_TYPE(msg_field_offset->oid) = NETPROX_SNMP_TYPE_OID;
		SNMP_FIELD_LEN(msg_field_offset->oid) =
			response_info->oid_total_depth;
		for (int i = 0; i < response_info->oid_total_depth; i++) {
			SNMP_FIELD_VALUE_BYTE(msg_field_offset->oid + i) =
				response_info->oid[i];
		}

		/* Get the offset of Value field */
		msg_field_offset->val =
			SNMP_NEXT_FIELD_OFFSET(msg_field_offset->oid) +
			SNMP_FIELD_LEN(msg_field_offset->oid);

		/* Fill in the type, length, and value of Value field */
		SNMP_FIELD_TYPE(msg_field_offset->val) =
			response_info->val_type;
		fr_snmp_fill_len(snmp_hdr, msg_field_offset->val,
				 response_info->val_len);
		for (int i = 0; i < response_info->val_len; i++) {
			SNMP_FIELD_LONG_FORM_VALUE_BYTE(msg_field_offset->val
							+ i) =
				*(response_info->val + i);
		}
	} else {
		/* Set NoSuchName error */
		SNMP_FIELD_VALUE_BYTE(msg_field_offset->err) =
			NETPROX_SNMP_ERROR_NOSUCHNAME;
		/* Only support GetNext or GetRequest, so error index
		 * is always the first OID.
		 */
		SNMP_FIELD_VALUE_BYTE(msg_field_offset->err_idx) = 1;
		add_len = 0;
	}

	pkt_len = frame->len + add_len;

	/* Set payload length of L3 and L4 */
	if (response_info->is_ipv6) {
		ip6_hdr->len = htons(pkt_len - sizeof(struct net_eth_hdr));
		udp_hdr->len = htons(pkt_len - sizeof(struct net_eth_hdr) -
				     sizeof(struct net_ipv6_hdr));
	} else {
		ip4_hdr->len = htons(pkt_len - sizeof(struct net_eth_hdr));
		udp_hdr->len = htons(pkt_len - sizeof(struct net_eth_hdr) -
				     sizeof(struct net_ipv4_hdr));
	}

	/* Transmit SNMP GetResponse message */
	np_ctx.net_ctx->np_eth_tx_data(np_ctx.net_ctx->iface,
				       (uint8_t *)eth_hdr, pkt_len);

	LOG_DBG("Send SNMP GetResponse message.");
}

/* Frame Classifier: Process SNMP message as shown below:
 *
 * SNMP v1 & v2c Message Format (TLV Format):-
 * =================================
 *     Type(1-byte)   Length(1-byte)    Value(L-byte)
 *  +---------------+---------------+
 *  |           SNMP Message        |
 *  +---------------+---------------+-------//--------+
 *  |                   SNMP Version                  |
 *  +---------------+---------------+-------//--------+
 *  |              SNMP Community String              |
 *  +---------------+---------------+-------//--------+
 *  |           SNMP PDU            |
 *  +---------------+---------------+-------//--------+
 *  |                   Request ID                    |
 *  +---------------+---------------+-------//--------+
 *  |                       Error                     |
 *  +---------------+---------------+-------//--------+
 *  |                   Error Index                   |
 *  +---------------+---------------+-------//--------+
 *  |          Varbind List         |
 *  +---------------+---------------+
 *  |            Varbind            |
 *  +---------------+---------------+-------//--------+
 *  |                Object Identifier                |
 *  +---------------+---------------+-------//--------+
 *  |                      Value                      |
 *  +---------------+---------------+-------//--------+
 */
static void fc_process_snmp(struct frame_classifier *frame,
			    struct net_udp_hdr *udp_hdr, bool is_ipv6)
{
	struct snmp_response_info response_info;
	struct snmp_msg_field_offset msg_field_offset;
	uint8_t *snmp_hdr;
	struct snmp_oid_tree_info *p_oid_info = &snmp_oid_tree_info;

	if (!(fc_rs_snmp & NP_RL_CLS_ENABLE)) {
		LOG_DBG("Ignoring SNMP msg: handling decision = disable.");
		return;
	}

	snmp_hdr = (uint8_t *)(udp_hdr + 1);

	/* Check SNMP version (Type=integer, Length=1, Value=v1/v2c)
	 * If check failed, silently drop the received SNMP message.
	 */
	msg_field_offset.ver = SNMP_NEXT_FIELD_OFFSET(0);
	if ((SNMP_FIELD_TYPE(msg_field_offset.ver) !=
	     NETPROX_SNMP_TYPE_INTEGER) ||
	    (SNMP_FIELD_LEN(msg_field_offset.ver) != 1) ||
	    ((SNMP_FIELD_VALUE_BYTE(msg_field_offset.ver) !=
	      NETPROX_SNMP_VERSION_V1) &&
	     (SNMP_FIELD_VALUE_BYTE(msg_field_offset.ver) !=
	      NETPROX_SNMP_VERSION_V2C))) {
		LOG_DBG("Ignoring SNMP msg: not matching version.");
		return;
	}

	/* Check community string
	 * If check failed, silently drop the received SNMP message.
	 */
	msg_field_offset.community_str =
		SNMP_NEXT_FIELD_OFFSET(msg_field_offset.ver) +
		SNMP_FIELD_LEN(msg_field_offset.ver);
	if ((SNMP_FIELD_LEN(msg_field_offset.community_str) !=
	     p_oid_info->community_str_sz) ||
	    strncmp(SNMP_FIELD_VALUE_PTR(msg_field_offset.community_str),
		    p_oid_info->community_str,
		    p_oid_info->community_str_sz)) {
		LOG_DBG("Ignoring SNMP msg: not matching community string.");
		return;
	}

	/* Get type of PDU
	 * If unsupported PDU type is obtained,
	 * silently drop the received SNMP message.
	 */
	msg_field_offset.pdu =
		SNMP_NEXT_FIELD_OFFSET(msg_field_offset.community_str) +
		SNMP_FIELD_LEN(msg_field_offset.community_str);
	switch (SNMP_FIELD_TYPE(msg_field_offset.pdu)) {
	case NETPROX_SNMP_PDU_SET:
		if (fc_rs_snmp & NP_RL_CLS_SUPP_SNMP_SET) {
			LOG_DBG("Wake Host to pass SNMP SetRequest message.");
			netprox_add_a2h_pkt_wake(frame->pkt, frame->len);
		} else {
			LOG_DBG("Ignoring SNMP SetRequest message.");
		}
		return;

	case NETPROX_SNMP_PDU_GET:
		response_info.get_next = 0;
		break;

	case NETPROX_SNMP_PDU_GET_NEXT:
		response_info.get_next = 1;
		break;

	default:
		LOG_DBG("Ignoring SNMP msg: not matching PDU type: %02X.",
			SNMP_FIELD_TYPE(msg_field_offset.pdu));
		return;
	}

	/* Get remaining SNMP message field offset */
	msg_field_offset.id = SNMP_NEXT_FIELD_OFFSET(msg_field_offset.pdu);
	msg_field_offset.err =
		SNMP_NEXT_FIELD_OFFSET(msg_field_offset.id) +
		SNMP_FIELD_LEN(msg_field_offset.id);
	msg_field_offset.err_idx =
		SNMP_NEXT_FIELD_OFFSET(msg_field_offset.err) +
		SNMP_FIELD_LEN(msg_field_offset.err);
	msg_field_offset.vbl =
		SNMP_NEXT_FIELD_OFFSET(msg_field_offset.err_idx) +
		SNMP_FIELD_LEN(msg_field_offset.err_idx);
	msg_field_offset.vb = SNMP_NEXT_FIELD_OFFSET(msg_field_offset.vbl);
	msg_field_offset.oid = SNMP_NEXT_FIELD_OFFSET(msg_field_offset.vb);
	msg_field_offset.val =
		SNMP_NEXT_FIELD_OFFSET(msg_field_offset.oid) +
		SNMP_FIELD_LEN(msg_field_offset.oid);

	/* Check last field (value) is termination (Type=Null, Length=0)
	 * If check failed, silently drop the received SNMP message.
	 */
	if ((SNMP_FIELD_TYPE(msg_field_offset.val) != NETPROX_SNMP_TYPE_NULL) ||
	    (SNMP_FIELD_LEN(msg_field_offset.val) != 0)) {
		LOG_DBG("Ignoring ill SNMP msg: last field (value) not equal "
			"to NULL.");
		return;
	}

	/* Check SNMP message length
	 * When length of last field (value) is equal to zero, the SNMP message
	 * length will be equal to offset of the value field.
	 * If check failed, silently drop the received SNMP message.
	 */
	if (SNMP_FIELD_LEN(0) != msg_field_offset.val) {
		LOG_DBG("Ignoring ill SNMP msg: msg length not tally.");
		return;
	}

	/* Get OID depth from received SNMP message */
	response_info.oid_total_depth = SNMP_FIELD_LEN(msg_field_offset.oid);

	if (response_info.oid_total_depth > NETPROX_SNMP_MAX_OID_DEPTH) {
		LOG_DBG("Ignoring SNMP msg: received OID depth (%d) is bigger "
			"than supported number (%d).",
			response_info.oid_total_depth,
			NETPROX_SNMP_MAX_OID_DEPTH);
		return;
	}

	/* Obtain OID from received SNMP message
	 * According to the design of OID tree which is passed from HOST:-
	 *  1) The leading two nodes of OID “1.3” is encoded using (40*x+y),
	 *     i.e. (40*1)+3=43 or 0x2b. The rest of the OID nodes are
	 *     encoded by using simple hexadecimal representation.
	 *  2) A 2-byte OID number (> 127) is treated as two different
	 *     OID nodes.
	 */
	for (int i = 0; i < response_info.oid_total_depth; i++) {
		response_info.oid[i] =
			SNMP_FIELD_VALUE_BYTE(msg_field_offset.oid + i);
	}

	response_info.is_ipv6 = is_ipv6;

	fr_process_snmp(frame, &response_info, &msg_field_offset);
}

/* fr_mdns_wr_cmprs_offset() - Write the two bytes compression offset to mDNS
 *			       response message.
 */
static void fr_mdns_wr_cmprs_offset(uint8_t **mdns_pos, uint16_t offset,
				    int *pkt_len)
{
	uint16_t cmprs_offset = MDNS_SET_CMPRS_OFFSET(offset);

	memcpy(*mdns_pos, &cmprs_offset, NETPROX_MDNS_QNAME_CT_CMPRS_OFFSET_SZ);
	*pkt_len += NETPROX_MDNS_QNAME_CT_CMPRS_OFFSET_SZ;
	*mdns_pos += NETPROX_MDNS_QNAME_CT_CMPRS_OFFSET_SZ;
}

/* fr_mdns_wr_content() - Write mDNS content which its position in mDNS database
 *			  is pointed by 'db_offset' to mDNS response message.
 */
static void fr_mdns_wr_content(uint8_t **mdns_pos, uint16_t db_offset, int len,
			       int *pkt_len)
{
	memcpy(*mdns_pos, &mdns_db.data[db_offset], len);
	*pkt_len += len;
	*mdns_pos += len;
}

/* fr_mdns_set_rr_name() - Write Answer RR name into mDNS response message.
 *
 * There are three cases in writing Answer RR name:
 *  a) The RR is the 1st RR in Answer section. In this case, 'ptr_rdata_offset'
 *     and 'construct->sld' are equal to 0. The uncompressed format of RR name
 *     is used in mDNS response message. Besides, the offset of top level domain
 *     (construct->tld) and second level domain (construct->sld) are recorded
 *     for message compression purpose.
 *
 *  b) The RR is not 1st RR in Answer section. In this case, 'ptr_rdata_offset'
 *     is equal to 0, whereas the offset of 1st RR's sld is recorded in
 *     'construct->sld'. The compressed format of RR name is used in mDNS
 *     response message. Besides, 'construct->sld' is used as compression offset
 *     of RR name.
 *     Assumption: All Answer RRs use the same second level domain
 *                 (e.g. _tcp.local).
 *
 *  c) The RR is TXT or SRV RR after a PTR RR. In this case. the offset of rdata
 *     of the PTR RR is recorded in 'ptr_rdata_offset'. The fully compressed
 *     format of RR name is used in mDNS response message with
 *     'ptr_rdata_offset' as the compression offset.
 */
static void fr_mdns_set_rr_name(struct mdns_name name,
				uint16_t ptr_rdata_offset,
				struct mdns_resp_construct_info *construct)
{
	uint16_t metadata_offset;

	if (ptr_rdata_offset) {
		fr_mdns_wr_cmprs_offset(&construct->mdns_pos, ptr_rdata_offset,
					&construct->pkt_len);
	} else if (construct->sld) {
		fr_mdns_wr_content(&construct->mdns_pos, name.val,
				   name.len[MDNS_CT_T_CMPRS],
				   &construct->pkt_len);
		fr_mdns_wr_cmprs_offset(&construct->mdns_pos, construct->sld,
					&construct->pkt_len);
	} else {
		fr_mdns_wr_content(&construct->mdns_pos, name.val,
				   name.len[MDNS_CT_T_FULL],
				   &construct->pkt_len);
		metadata_offset = construct->pkt_len - construct->hdr;
		construct->tld = MDNS_GET_TLD_OFFSET(metadata_offset);
		construct->sld = MDNS_GET_SLD_OFFSET(metadata_offset);
	}
}

/* fr_mdns_set_a_rr() - Write Answer RR for A query into Answer section of mDNS
 *			response message.
 *
 * Note:
 *  a) Name of A RR is always fully compressed with offset = SRV RR's target.
 */
static void fr_mdns_set_a_rr(uint16_t entry_index, uint16_t srv_target_offset,
			     uint8_t **mdns_pos, int *pkt_len)
{
	struct mdns_a_info *a_info;

	/* Get A RR information structure from DB */
	a_info = MDNS_A_INFO_PTR(entry_index);

	/* Write A Answer RR name (in fully compressed format) into
	 * mDNS response message
	 */
	fr_mdns_wr_cmprs_offset(mdns_pos, srv_target_offset, pkt_len);

	/* Write A Answer RR metadata into mDNS response message */
	fr_mdns_wr_content(mdns_pos, a_info->metadata,
			   NETPROX_MDNS_METADATA_SZ, pkt_len);

	/* Write A Answer RR rdata (IPv4 addr) into mDNS response message */
	fr_mdns_wr_content(mdns_pos, a_info->rdata, NP_IPV4_ADDR_BYTES,
			   pkt_len);
}

/* fr_mdns_set_aaaa_rr() - Write Answer RR for AAAA query into Answer section of
 *			   mDNS response message.
 *
 * Note:
 *  a) Name of AAAA RR is always fully compressed with offset = SRV RR's target.
 */
static void fr_mdns_set_aaaa_rr(uint16_t entry_index,
				uint16_t srv_target_offset,
				uint8_t **mdns_pos, int *pkt_len)
{
	struct mdns_aaaa_info *aaaa_info;

	/* Get AAAA RR information structure from DB */
	aaaa_info = MDNS_AAAA_INFO_PTR(entry_index);

	/* Write AAAA Answer RR name (in fully compressed format) into
	 * mDNS response message
	 */
	fr_mdns_wr_cmprs_offset(mdns_pos, srv_target_offset, pkt_len);

	/* Write AAAA Answer RR metadata into mDNS response message */
	fr_mdns_wr_content(mdns_pos, aaaa_info->metadata,
			   NETPROX_MDNS_METADATA_SZ, pkt_len);

	/* Write AAAA Answer RR rdata (IPv6 addr) into mDNS response message */
	fr_mdns_wr_content(mdns_pos, aaaa_info->rdata, NP_IPV6_ADDR_BYTES,
			   pkt_len);
}

/* fr_mdns_set_srv_rr() - Write Answer RRs for SRV query into Answer section of
 *			  mDNS response message.
 *
 * The Answer RRs for a SRV query are shown in diagram below:
 *
 *    SRV RR [name, metadata, rdata (target:*1)]
 *     |
 *     |- AAAA (IPv6) RR [name (fully cmprs to *1), metadata, rdata]
 *     |
 *     |- A (IPv4) RR [name (fully cmprs to *1), metadata, rdata]
 *
 * Note:
 *  a) Name of SRV RR will be compressed/uncompressed according to its position
 *     in mDNS response message that varies across different mDNS queries:
 *       i) PTR query:
 *          SRV RR 'name' is after PTR RR in mDNS response. Therefore, SRV RR
 *          'name' is fully compressed with offset = PTR RR's rdata.
 *      ii) SRV query:
 *          a) SRV RR 'name' is in the 1st RR and uncompressed.
 *          b) SRV RR 'name' is not the 1st RR and partially compressed with
 *             offset = 1st RR's SLD.
 *          Assumption: All Answer RRs for SRV query use the same second level
 *                      domain (e.g. _tcp.local).
 *  b) Target of SRV RR is partially compressed with offset = TLD of 1st Answer
 *     RR.
 *  c) A (IPv4) RR is needed only if IPv4 mDNS query message is received.
 */
static void fr_mdns_set_srv_rr(uint16_t entry_index, uint16_t ptr_rdata_offset,
			       struct mdns_resp_construct_info *construct)
{
	struct mdns_srv_info *srv_info;
	uint16_t srv_target_offset;
	uint16_t hdr = construct->hdr;

	/* Get SRV RR information structure from DB */
	srv_info = MDNS_SRV_INFO_PTR(entry_index);

	/* Write SRV Answer RR name into mDNS response message */
	fr_mdns_set_rr_name(srv_info->name, ptr_rdata_offset, construct);

	fr_mdns_wr_content(&construct->mdns_pos, srv_info->metadata,
			   NETPROX_MDNS_METADATA_SZ, &construct->pkt_len);

	/* The name of SRV corresponding AAAA and A RRs will be fully compressed
	 * to SRV target, which its offset is 'srv_target_offset.
	 */
	srv_target_offset = MDNS_GET_SRV_TARGET_OFFSET(construct->pkt_len
						       - hdr);

	fr_mdns_wr_content(&construct->mdns_pos, srv_info->rdata,
			   srv_info->rdlen[MDNS_CT_T_CMPRS],
			   &construct->pkt_len);

	fr_mdns_wr_cmprs_offset(&construct->mdns_pos, construct->tld,
				&construct->pkt_len);

	construct->ancount += 1;

	fr_mdns_set_aaaa_rr(srv_info->aaaa, srv_target_offset,
			    &construct->mdns_pos, &construct->pkt_len);
	construct->ancount += 1;

	/* A (IPv4) RR is needed only if IPv4 mDNS query message is received. */
	if (!construct->is_ipv6) {
		fr_mdns_set_a_rr(srv_info->a, srv_target_offset,
				 &construct->mdns_pos, &construct->pkt_len);
		construct->ancount += 1;
	}
}

/* fr_mdns_set_txt_rr() - Write Answer RR for TXT query into Answer section of
 *			  mDNS response message.
 *
 * Name of TXT RR is compressed/uncompressed according to its position in mDNS
 * response message that varies across different mDNS queries:
 *   i) PTR query:
 *      TXT RR 'name' is after PTR RR in mDNS response. Therefore, TXT RR
 *      'name' is fully compressed with offset = PTR RR's rdata.
 *  ii) TXT query:
 *      a) TXT RR 'name' is in the 1st RR and uncompressed.
 *      b) TXT RR 'name' is not the 1st RR and partially compressed with
 *         offset = 1st RR's SLD.
 *         Assumption: All Answer RRs for TXT query use the same second level
 *                     domain (e.g. _tcp.local).
 */
static void fr_mdns_set_txt_rr(uint16_t entry_index, uint16_t ptr_rdata_offset,
			       struct mdns_resp_construct_info *construct)
{
	struct mdns_txt_info *txt_info;

	/* Get TXT RR information structure from DB */
	txt_info = MDNS_TXT_INFO_PTR(entry_index);

	fr_mdns_set_rr_name(txt_info->name, ptr_rdata_offset, construct);

	/* Write TXT Answer RR metadata into mDNS response message */
	fr_mdns_wr_content(&construct->mdns_pos, txt_info->metadata,
			   NETPROX_MDNS_METADATA_SZ, &construct->pkt_len);

	/* Write TXT Answer RR rdata into mDNS response message */
	fr_mdns_wr_content(&construct->mdns_pos, txt_info->rdata,
			   txt_info->rdlen, &construct->pkt_len);

	construct->ancount += 1;
}

/* fr_mdns_set_ptr_rr() - Write Answer RRs for PTR query into Answer section of
 *			  mDNS response message.
 *
 * The Answer RRs for a PTR query are shown in diagram below:
 *
 *    PTR RR [name, metadata, rdata (*1)]
 *     |
 *     |- TXT RR [name (fully cmprs to *1), metadata, rdata]
 *     |
 *     |- SRV RR [name (fully cmprs to *1), metadata, rdata (target:*2)]
 *         |
 *         |- AAAA (IPv6) RR [name (fully cmprs to *2), metadata, rdata]
 *         |
 *         |- A (IPv4) RR [name (fully cmprs to *2), metadata, rdata]
 *
 * Note:
 *  a) Target of SRV RR is partially compressed with offset = TLD of 1st Answer
 *     RR (e.g. .local).
 *  b) A (IPv4) RR is needed only if IPv4 mDNS query message is received.
 *  c) Refer to RFC1035 Section 4.1.4 for understanding of message compression
 *     technique.
 */
static void fr_mdns_set_ptr_rr(struct mdns_resp_rr_info rr,
			       struct mdns_resp_construct_info *construct)
{
	struct mdns_ptr_info *ptr_info;
	uint16_t ptr_rdata_offset;
	uint16_t ptr_3ld_offset;

	/* Get PTR RR information structure from DB */
	ptr_info = MDNS_PTR_INFO_PTR(rr.rr_entry_index);

	/* Get offset of third level domain */
	ptr_3ld_offset = construct->pkt_len
			 + ptr_info->name[rr.ptr_t].len[MDNS_CT_T_FULL]
			 - ptr_info->rdlen[MDNS_CT_T_FULL]
			 + ptr_info->rdlen[MDNS_CT_T_CMPRS] - construct->hdr;

	/* Write PTR Answer RR name into mDNS response message */
	fr_mdns_set_rr_name(ptr_info->name[rr.ptr_t], 0, construct);

	/* Write PTR Answer RR metadata into mDNS response message */
	fr_mdns_wr_content(&construct->mdns_pos, ptr_info->metadata,
			   NETPROX_MDNS_METADATA_SZ, &construct->pkt_len);

	/* Get the offset of PTR rdata
	 * 'ptr_rdata_offset' is the position where fully compressed TXT and SRV
	 * RR name will point to.
	 */
	ptr_rdata_offset = construct->pkt_len - construct->hdr;

	/* Write PTR Answer RR rdata (in compressed format) into mDNS
	 * response message
	 */
	fr_mdns_wr_content(&construct->mdns_pos, ptr_info->rdata,
			   ptr_info->rdlen[MDNS_CT_T_CMPRS],
			   &construct->pkt_len);
	fr_mdns_wr_cmprs_offset(&construct->mdns_pos, ptr_3ld_offset,
				&construct->pkt_len);

	construct->ancount += 1;

	/* PTR Answer RR will be followed by TXT and SRV RR */
	fr_mdns_set_txt_rr(ptr_info->txt, ptr_rdata_offset, construct);
	fr_mdns_set_srv_rr(ptr_info->srv, ptr_rdata_offset, construct);
}

/* fr_mdns_set_dnssd_rr() - Write Answer RRs for DNS-SD PTR query into Answer
 *			    section of mDNS response message.
 */
static void fr_mdns_set_dnssd_rr(uint16_t entry_index,
				 struct mdns_resp_construct_info *construct)
{
	struct mdns_dnssd_ptr_info *dnssd_info;

	/* Get DNS-SD PTR RR information structure from DB */
	dnssd_info = MDNS_DNSSD_INFO_PTR(entry_index);

	/* Write DNS-SD PTR Answer RR into mDNS response message */
	fr_mdns_wr_content(&construct->mdns_pos, dnssd_info->answer,
			   dnssd_info->answer_len, &construct->pkt_len);

	construct->ancount += dnssd_info->ancount;
}

/* Frame Responder: Process mDNS response message */
static void fr_process_mdns(struct frame_classifier *frame,
			    struct mdns_resp_info resp_info)
{
	struct net_eth_hdr *eth_hdr;
	struct net_ipv4_hdr *ip4_hdr = NULL;
	struct net_ipv6_hdr *ip6_hdr = NULL;
	struct net_udp_hdr *udp_hdr;
	struct mdns_hdr *mdns_hdr;
	struct in_addr *host_addr4;
	struct in6_addr *host_addr6;
	uint16_t chksum = 0;
	bool is_ipv6 = resp_info.construct.is_ipv6;
	bool ptr_resp = 0;
	int i;
	uint32_t wait_time = 0;

	/* Get L2, L3 and L4 headers */
	eth_hdr = (struct net_eth_hdr *)frame->pkt;
	if (is_ipv6) {
		ip6_hdr = (struct net_ipv6_hdr *)(eth_hdr + 1);
		udp_hdr = (struct net_udp_hdr *)(ip6_hdr + 1);
	} else {
		ip4_hdr = (struct net_ipv4_hdr *)(eth_hdr + 1);
		udp_hdr = (struct net_udp_hdr *)(ip4_hdr + 1);
		ip4_hdr->chksum = 0;
		ip4_hdr->ttl = NETRPOX_MDNS_DEFAULT_IPV4_TTL;
	}
	udp_hdr->chksum = 0;
	mdns_hdr = (struct mdns_hdr *)(udp_hdr + 1);

	/* According to RFC 6762 Section 5.4, when unicast-response bit is set
	 * in a question, it indicates that the querier is willing to accept
	 * unicast as well as multicast replies in response. Therefore, we will
	 * always respond mDNS query with multicast DNS regardless of the
	 * unicast-response bit is set or not; that is also the common practice
	 * observed in printing service discovery over mDNS. Therefore, we only
	 * change the source MAC and IP addresses but maintain the destination
	 * MAC and IP addresses as mDNS multicast addresses.
	 */
	memcpy(&eth_hdr->src, np_mib.mac_addr, NP_MAC_ADDR_BYTES);
	if (is_ipv6) {
		host_addr6 = (struct in6_addr *)np_mib.ipv6_addr[0];
		net_ipaddr_copy(&ip6_hdr->src, host_addr6);
	} else {
		host_addr4 = (struct in_addr *)np_mib.ipv4_addr;
		net_ipaddr_copy(&ip4_hdr->src, host_addr4);
	}

	/* Setup mDNS header message */
	mdns_hdr->id = 0;
	mdns_hdr->flags = htons(NETPROX_MDNS_HDR_FLAGS_QR
				| NETPROX_MDNS_HDR_FLAGS_AA);
	mdns_hdr->qdcount = 0;
	mdns_hdr->ancount = 0;
	mdns_hdr->nscount = 0;
	mdns_hdr->arcount = 0;

	if (is_ipv6) {
		resp_info.construct.pkt_len = sizeof(struct net_ipv6_hdr);
	} else {
		resp_info.construct.pkt_len = sizeof(struct net_ipv4_hdr);
	}

	resp_info.construct.pkt_len += sizeof(struct net_eth_hdr) +
				       sizeof(struct net_udp_hdr);

	resp_info.construct.hdr = resp_info.construct.pkt_len;

	resp_info.construct.pkt_len += sizeof(struct mdns_hdr);

	resp_info.construct.mdns_pos = (uint8_t *)(mdns_hdr + 1);

	/* Write the required Answer RRs into mDNS response message */
	for (i = 0; i < resp_info.need_ans; i++) {
		switch (resp_info.rr[i].rr_type) {
		case MDNS_RR_T_DNSSD_PTR:
			fr_mdns_set_dnssd_rr(resp_info.rr[i].rr_entry_index,
					     &resp_info.construct);
			ptr_resp = 1;
			break;

		case MDNS_RR_T_PTR:
			fr_mdns_set_ptr_rr(resp_info.rr[i],
					   &resp_info.construct);
			ptr_resp = 1;
			break;

		case MDNS_RR_T_TXT:
			fr_mdns_set_txt_rr(resp_info.rr[i].rr_entry_index, 0,
					   &resp_info.construct);
			break;

		case MDNS_RR_T_SRV:
			fr_mdns_set_srv_rr(resp_info.rr[i].rr_entry_index, 0,
					   &resp_info.construct);
			break;

		default:
			break;
		}
	}

	/* Set payload length of L3 and L4 */
	if (is_ipv6) {
		ip6_hdr->len = htons(resp_info.construct.pkt_len
				     - sizeof(struct net_eth_hdr)
				     - sizeof(struct net_ipv6_hdr));
		udp_hdr->len = ip6_hdr->len;
	} else {
		ip4_hdr->len = htons(resp_info.construct.pkt_len
				     - sizeof(struct net_eth_hdr));
		udp_hdr->len = htons(resp_info.construct.pkt_len
				     - sizeof(struct net_eth_hdr)
				     - sizeof(struct net_ipv4_hdr));
	}

	mdns_hdr->ancount = htons(resp_info.construct.ancount);

	/* Change cheksum to 0 to calculate new cheksum */
	udp_hdr->chksum = 0;
	chksum = netprox_calc_chksum(0, (uint8_t *)udp_hdr, udp_hdr->len);
	udp_hdr->chksum = ~htons(chksum);

	/* Wait a random interval of time before response to PTR query */
	if (ptr_resp) {
		wait_time = sys_rand32_get() & NETPROX_MDNS_PTR_WAIT_TIME_MASK;
		k_sleep(K_MSEC(wait_time));
	}

	/* Transmit mDNS Response message */
	np_ctx.net_ctx->np_eth_tx_data(np_ctx.net_ctx->iface,
				       (uint8_t *)eth_hdr,
				       resp_info.construct.pkt_len);
	LOG_DBG("Send mDNS response message.");
}

/* fc_mdns_check_conflit() - Check whether there is conflit resolution in the
 *			     received known-answer.
 */
static int fc_mdns_check_conflit(struct frame_classifier *frame,
				 struct mdns_answer_info *ans_info,
				 struct mdns_name dbname, uint16_t index,
				 uint16_t dbrdata, uint16_t dbrdlen,
				 struct mdns_resp_info *resp_info)
{
	int rr_need_move_forward;

	if (dbname.len[MDNS_CT_T_FULL] != ans_info->name_len) {
		return 0;
	}

	if (memcmp(&mdns_db.data[dbname.val], ans_info->name,
		   ans_info->name_len) != 0) {
		return 0;
	}

	if (dbrdlen != ans_info->rdlen) {
		LOG_DBG("mDNS wake host for conflit resolution.");
		netprox_add_a2h_pkt_wake(frame->pkt, frame->len);
		return MDNS_KNOWN_ANS_CONFLIT_FOUND;
	}

	if (memcmp(&mdns_db.data[dbrdata], ans_info->rdata, ans_info->rdlen) !=
	    0) {
		LOG_DBG("mDNS wake host for conflit resolution.");
		netprox_add_a2h_pkt_wake(frame->pkt, frame->len);
		return MDNS_KNOWN_ANS_CONFLIT_FOUND;
	}

	/* Remove known-answer response from resp_info->rr[] and move forward
	 * the remaining 'resp_info->rr[]'.
	 */
	resp_info->need_ans -= 1;
	rr_need_move_forward = resp_info->need_ans - index;
	memcpy(&resp_info->rr[index], &resp_info->rr[index + 1],
	       sizeof(struct mdns_resp_rr_info) * rr_need_move_forward);

	return MDNS_KNOWN_ANS_DATA_MATCHED;
}

/* fc_mdns_get_domain_name() - Get name in FQDN (Fully Qualified Domain
 *			       Name, i.e. uncompressed) format.
 *
 * This function will keep trace the domain name according to 'label_len'. If
 * message compression is found, it will jump to the position pointed by
 * 'cmprs_offset' and continue to trace until zero-length NULL is reached.
 */
static int fc_mdns_get_domain_name(uint8_t *mdns_hdr_start, uint8_t **mdns_pos,
				   uint8_t *name, uint16_t *name_len)
{
	uint8_t *p_name;
	uint8_t prefix;
	uint8_t label_len;
	uint8_t next_subfield = 1;
	bool f_cmprs = 0;
	uint16_t cmprs_offset;
	uint16_t subfield_len;

	/* Check NULL pointer */
	if (!mdns_hdr_start | !*mdns_pos | !name | !name_len) {
		return -1;
	}

	p_name = *mdns_pos;

	while (next_subfield) {
		prefix = *p_name & NETPROX_MDNS_QNAME_CT_MASK;

		switch (prefix) {
		case NETPROX_MDNS_QNAME_CT_LABEL:
			/* Get total length of length-value subfield */
			label_len = *p_name;
			subfield_len = label_len +
				       NETPROX_MDNS_QNAME_CT_LABEL_LEN_SZ;

			/* Copy label_len and label */
			memcpy(name + *name_len, p_name, subfield_len);
			*name_len += subfield_len;
			p_name += subfield_len;

			/* If label_len == 0, means zero-length NULL is reached.
			 * In this case, we will stop the while loop since the
			 * end of domain name is reached.
			 */
			next_subfield = label_len;

			/* Update the position of next item pointer according
			 * to the obtained length of subfield.
			 */
			if (!f_cmprs) {
				*mdns_pos += subfield_len;
			}
			break;

		case NETPROX_MDNS_QNAME_CT_CMPRS:
			/* Get the 14 bits compression offset */
			cmprs_offset =
				MDNS_GET_UPPER_BYTE_CMPRS_OFFSET(*p_name);
			cmprs_offset += *(p_name + 1);

			/* Reposition the p_name according to the compression
			 * offset and continue the while loop so that FQDN can
			 * be obtained.
			 */
			p_name = mdns_hdr_start + cmprs_offset;
			next_subfield = 1;

			/* Set the compression flag so that next item pointer
			 * will not be updated.
			 */
			if (!f_cmprs) {
				*mdns_pos +=
					NETPROX_MDNS_QNAME_CT_CMPRS_OFFSET_SZ;
				f_cmprs = 1;
			}
			break;

		default:
			LOG_DBG("Ignoring ill mDNS msg: unrecognized prefix of "
				"label : %02x.", prefix);
			return -1;
		}
	}

	return 0;
}

/* fc_mdns_check_known_ans() - Check rdata of received known-answer.
 *
 * This function will
 *  a) suspress known-answer response.
 *  b) wake-up Host upon conflit resolution.
 */
static int fc_mdns_check_known_ans(struct frame_classifier *frame,
				   uint8_t *mdns_hdr_start, uint8_t **mdns_pos,
				   struct mdns_resp_info *resp_info)
{
	struct mdns_answer_info ans_info = { 0 };
	struct mdns_ptr_info *ptr_info;
	struct mdns_txt_info *txt_info;
	struct mdns_srv_info *srv_info;
	uint16_t srv_rdlen;
	int ret;
	int i;

	/* Get Answer name */
	ret = fc_mdns_get_domain_name(mdns_hdr_start, mdns_pos, ans_info.name,
				      &ans_info.name_len);
	if (ret) {
		LOG_DBG("Ignoring ill mDNS msg: unable to get known-answer "
			"name.");
		return ret;
	}

	/* Get Answer metadata */
	ans_info.type = ntohs(*(uint16_t *)(*mdns_pos));
	*mdns_pos += NETPROX_MDNS_TYPE_SZ;
	ans_info.class = ntohs(*(uint16_t *)(*mdns_pos));
	*mdns_pos += NETPROX_MDNS_CLASS_SZ;
	ans_info.ttl = ntohl(*(uint32_t *)(*mdns_pos));
	*mdns_pos += NETPROX_MDNS_TTL_SZ;

	/* Get Answer rdata in uncompressed format */
	switch (ans_info.type) {
	case NETPROX_MDNS_QTYPE_PTR:
		ans_info.type = MDNS_RR_T_PTR;
		*mdns_pos += NETPROX_MDNS_RDLENGTH_SZ;
		ret = fc_mdns_get_domain_name(mdns_hdr_start, mdns_pos,
					      ans_info.rdata, &ans_info.rdlen);
		if (ret) {
			LOG_DBG("Ignoring ill mDNS msg: unable to get PTR "
				"known-answer name.");
			return ret;
		}
		break;

	case NETPROX_MDNS_QTYPE_TXT:
		ans_info.type = MDNS_RR_T_TXT;
		ans_info.rdlen = ntohs(*(uint16_t *)(*mdns_pos));
		*mdns_pos += NETPROX_MDNS_RDLENGTH_SZ;
		memcpy(ans_info.rdata, *mdns_pos, ans_info.rdlen);
		*mdns_pos += ans_info.rdlen;
		break;

	case NETPROX_MDNS_QTYPE_SRV:
		ans_info.type = MDNS_RR_T_SRV;
		ans_info.rdlen = NETPROX_MDNS_SRV_PWP_SZ;
		*mdns_pos += NETPROX_MDNS_RDLENGTH_SZ;
		memcpy(ans_info.rdata, *mdns_pos, ans_info.rdlen);
		*mdns_pos += NETPROX_MDNS_SRV_PWP_SZ;
		ret = fc_mdns_get_domain_name(mdns_hdr_start, mdns_pos,
					      ans_info.rdata, &ans_info.rdlen);
		if (ret) {
			LOG_DBG("Ignoring ill mDNS msg: unable to get SRV "
				"known-answer target.");
			return ret;
		}
		break;

	default:
		return 0;
	}

	/* Check Internet Class */
	if ((ans_info.class & ~NETPROX_MDNS_QCLASS_QU) !=
	    NETPROX_MDNS_QCLASS_IN) {
		return 0;
	}

	/* TODO: Check Answer TTL */

	/* Check conflit resolution */
	for (i = 0; i < resp_info->need_ans; i++) {
		struct mdns_resp_rr_info resp = resp_info->rr[i];

		if (resp.rr_type != ans_info.type) {
			continue;
		}

		switch (ans_info.type) {
		case MDNS_RR_T_PTR:
			ptr_info = MDNS_PTR_INFO_PTR(resp.rr_entry_index);

			ret = fc_mdns_check_conflit(frame, &ans_info,
						    ptr_info->name[resp.ptr_t],
						    i, ptr_info->rdata,
						    ptr_info->rdlen[
							    MDNS_CT_T_FULL],
						    resp_info);
			if (ret == MDNS_KNOWN_ANS_CONFLIT_FOUND) {
				return ret;
			} else if (ret == MDNS_KNOWN_ANS_DATA_MATCHED) {
				return 0;
			}
			break;

		case MDNS_RR_T_TXT:
			txt_info = MDNS_TXT_INFO_PTR(resp.rr_entry_index);

			ret = fc_mdns_check_conflit(frame, &ans_info,
						    txt_info->name, i,
						    txt_info->rdata,
						    txt_info->rdlen, resp_info);
			if (ret == MDNS_KNOWN_ANS_CONFLIT_FOUND) {
				return ret;
			} else if (ret == MDNS_KNOWN_ANS_DATA_MATCHED) {
				return 0;
			}
			break;

		case MDNS_RR_T_SRV:
			srv_info = MDNS_SRV_INFO_PTR(resp.rr_entry_index);

			srv_rdlen = srv_info->rdlen[MDNS_CT_T_FULL];

			ret = fc_mdns_check_conflit(frame, &ans_info,
						    srv_info->name, i,
						    srv_info->rdata,
						    srv_rdlen, resp_info);
			if (ret == MDNS_KNOWN_ANS_CONFLIT_FOUND) {
				return ret;
			} else if (ret == MDNS_KNOWN_ANS_DATA_MATCHED) {
				return 0;
			}
			break;

		default:
			break;
		}
	}

	return 0;
}

/* fc_mdns_compare_name() - Compare the received query name with the recorded
 *			    name in mDNS database.
 *
 * If the query name is matched, add the rr_entry_index and its corresponding
 * rr_type into 'resp_info' and return 1. Otherwise, return 0.
 */
static int fc_mdns_compare_name(uint8_t *qname, uint16_t qname_len,
				uint16_t rr_type, struct mdns_name dbname,
				uint16_t rr_entry_index,
				struct mdns_resp_info *resp_info)
{
	if (dbname.len[MDNS_CT_T_FULL] != qname_len) {
		return 0;
	}

	if (memcmp(&mdns_db.data[dbname.val], qname, qname_len) != 0) {
		return 0;
	}

	resp_info->rr[resp_info->need_ans].rr_entry_index = rr_entry_index;
	resp_info->rr[resp_info->need_ans].rr_type = rr_type;
	resp_info->need_ans += 1;

	return 1;
}

/* fc_mdns_compare_query() - Check whether mDNS database contains the received
 *			     mDNS query.
 */
static void fc_mdns_compare_query(struct mdns_question_info *query_info,
				  struct mdns_resp_info *resp_info)
{
	uint16_t qname_len = query_info->name_len;
	uint8_t *qname = query_info->name;
	struct mdns_dnssd_ptr_info *dnssd_info;
	struct mdns_ptr_info *ptr_info;
	struct mdns_txt_info *txt_info;
	struct mdns_srv_info *srv_info;
	int i, j;

	switch (query_info->type) {
	case NETPROX_MDNS_QTYPE_PTR:
		for (i = 0; i < MDNS_GET_RR_COUNT(MDNS_RR_T_DNSSD_PTR); i++) {
			dnssd_info = MDNS_DNSSD_INFO_PTR(i);
			if (fc_mdns_compare_name(qname, qname_len,
						 MDNS_RR_T_DNSSD_PTR,
						 dnssd_info->name, i,
						 resp_info)) {
				return;
			}
		}

		for (i = 0; i < MDNS_GET_RR_COUNT(MDNS_RR_T_PTR); i++) {
			ptr_info = MDNS_PTR_INFO_PTR(i);
			for (j = 0; j < MDNS_PTR_T_MAX; j++) {
				if (fc_mdns_compare_name(qname, qname_len,
							 MDNS_RR_T_PTR,
							 ptr_info->name[j], i,
							 resp_info)) {
					resp_info->rr[resp_info->need_ans -
						      1].ptr_t = j;
					return;
				}
			}
		}
		break;

	case NETPROX_MDNS_QTYPE_TXT:
		for (i = 0; i < MDNS_GET_RR_COUNT(MDNS_RR_T_TXT); i++) {
			txt_info = MDNS_TXT_INFO_PTR(i);
			if (fc_mdns_compare_name(qname, qname_len,
						 MDNS_RR_T_TXT, txt_info->name,
						 i, resp_info)) {
				return;
			}
		}
		break;

	case NETPROX_MDNS_QTYPE_SRV:
		for (i = 0; i < MDNS_GET_RR_COUNT(MDNS_RR_T_SRV); i++) {
			srv_info = MDNS_SRV_INFO_PTR(i);
			if (fc_mdns_compare_name(qname, qname_len,
						 MDNS_RR_T_SRV, srv_info->name,
						 i, resp_info)) {
				return;
			}
		}
		break;

	default:
		break;
	}
}

/* fc_mdns_check_query() - Check QNAME, QTYPE, and QCLASS against mDNS database.
 *
 * mDNS Message Question Section Format :-
 * =======================================
 *  00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 * |                                               |
 * /                     QNAME                     /
 * |                                               |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 * |                     QTYPE                     |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 * |QU|                  QCLASS                    |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 */
static int fc_mdns_check_query(uint8_t *mdns_hdr_start, uint8_t **mdns_pos,
			       struct mdns_resp_info *resp_info)
{
	struct mdns_question_info query_info = { 0 };
	int ret;

	/* Get QNAME in FQDN (uncompressed) format */
	ret = fc_mdns_get_domain_name(mdns_hdr_start, mdns_pos,
				      query_info.name, &query_info.name_len);
	if (ret) {
		LOG_DBG("Ignoring ill mDNS msg: unable to get query name.");
		return ret;
	}

	/* Get QTYPE and QCLASS */
	query_info.type = ntohs(*(uint16_t *)(*mdns_pos));
	*mdns_pos += NETPROX_MDNS_TYPE_SZ;
	query_info.class = ntohs(*(uint16_t *)(*mdns_pos));
	*mdns_pos += NETPROX_MDNS_CLASS_SZ;

	/* Check Internet Class */
	if ((query_info.class & ~NETPROX_MDNS_QCLASS_QU) !=
	    NETPROX_MDNS_QCLASS_IN) {
		LOG_DBG("Ignoring mDNS msg: not matching QCLASS: %04x.",
			query_info.class);
		return -1;
	}

	/* Check the existence of received query from mDNS database */
	if (resp_info->need_ans < NETPROX_MDNS_QUERY_TO_BE_HANDLE_MAX) {
		fc_mdns_compare_query(&query_info, resp_info);
	}

	return ret;
}

/* Frame Classifier: Process mDNS message according to format shown below:
 *
 * mDNS Message Format :-
 * ======================
 * The top level of mDNS message format is divided
 * into 5 sections, as shown below:
 * +-----------------------------------------------+
 * |                     Header                    |
 * +-----------------------------------------------+
 * |                    Question                   |
 * +-----------------------------------------------+
 * |                     Answer                    |
 * +-----------------------------------------------+
 * |                   Authority                   |
 * +-----------------------------------------------+
 * |                   Additional                  |
 * +-----------------------------------------------+
 */
static void fc_process_mdns(struct frame_classifier *frame, bool is_ipv6)
{
	struct net_eth_hdr *eth_hdr;
	struct net_ipv4_hdr *ip4_hdr = NULL;
	struct net_ipv6_hdr *ip6_hdr = NULL;
	struct net_udp_hdr *udp_hdr;
	struct mdns_hdr *mdns_hdr;
	struct mdns_resp_info resp_info = { 0 };
	uint8_t *mdns_hdr_start;
	uint8_t *mdns_pos;
	int i;

	/* Check mDNS message handling decision */
	if (!(fc_rs_mdns & NP_RL_CLS_ENABLE)) {
		LOG_DBG("Ignoring mDNS msg: handling decision = disable.");
		return;
	}

	/* Get L2, L3, and L4 header */
	eth_hdr = (struct net_eth_hdr *)frame->pkt;
	if (is_ipv6) {
		ip6_hdr = (struct net_ipv6_hdr *)(eth_hdr + 1);

		/* Check mDNS multicast IPv6 address */
		struct in6_addr *host_addr;

		host_addr = (struct in6_addr *)mdns_db_hdr->ipv6_maddr;
		if (!net_ipv6_addr_cmp(host_addr, &ip6_hdr->dst)) {
			LOG_DBG("Ignoring mDNS msg: not matching destination "
				"IPv6 address.");
			return;
		}

		/* Check UDP Protocol */
		if (ip6_hdr->nexthdr != IPPROTO_UDP) {
			LOG_DBG("Ignoring mDNS msg: not matching IPv6 "
				"next-header.");
			return;
		}

		udp_hdr = (struct net_udp_hdr *)(ip6_hdr + 1);
		resp_info.construct.is_ipv6 = 1;
	} else {
		struct in_addr *host_addr;
		uint32_t host_subnet;
		uint32_t src_subnet;

		ip4_hdr = (struct net_ipv4_hdr *)(eth_hdr + 1);

		/* Check mDNS multicast IPv4 address */
		host_addr = (struct in_addr *)mdns_db_hdr->ipv4_maddr;
		if (!net_ipv4_addr_cmp(host_addr, &ip4_hdr->dst)) {
			LOG_DBG("Ignoring mDNS msg: not matching destination "
				"IPv4 address.");
			return;
		}

		/* Check IPv4 source address */
		host_subnet = *(uint32_t *)np_mib.ipv4_addr &
			      *(uint32_t *)np_mib.ipv4_subnet;
		src_subnet = ip4_hdr->src.s_addr & *(uint32_t *)np_mib.ipv4_subnet;
		if (!net_ipv4_addr_cmp((struct in_addr *)&host_subnet,
				       (struct in_addr *)&src_subnet)) {
			LOG_DBG("Ignoring mDNS msg: not matching source IPv4 "
				"address.");
			return;
		}

		/* Check UDP Protocol */
		if (ip4_hdr->proto != IPPROTO_UDP) {
			LOG_DBG("Ignoring mDNS msg: not matching IPv4 "
				"protocol.");
			return;
		}

		udp_hdr = (struct net_udp_hdr *)(ip4_hdr + 1);
		resp_info.construct.is_ipv6 = 0;
	}

	/* Check UDP port */
	if (udp_hdr->dst_port != htons(NETPROX_MDNS_DEFAULT_UDP_PORT)) {
		LOG_DBG("Ignoring mDNS msg: not matching UDP destination "
			"port.");
		return;
	}

	/* Get mDNS message header */
	mdns_hdr = (struct mdns_hdr *)(udp_hdr + 1);

	/* Check mDNS message type */
	if (ntohs(mdns_hdr->flags) & NETPROX_MDNS_HDR_FLAGS_QR) {
		LOG_DBG("Ignoring mDNS response msg.");
		return;
	}

	/* Check existence of truncated mDNS message that will have additional
	 * known-answer in following message
	 */
	if (ntohs(mdns_hdr->flags) & NETPROX_MDNS_HDR_FLAGS_TC) {
		/* TODO: Add support to truncated mDNS query */
		LOG_DBG("Ignoring mDNS msg: Truncated bit is set.");
		return;
	}

	/* Get the start position of mDNS header for message uncompression */
	mdns_hdr_start = (uint8_t *)mdns_hdr;

	/* Point mdns_pos to position after mDNS header */
	mdns_pos = (uint8_t *)(mdns_hdr + 1);

	/* Get relevant queries from received mDNS message */
	for (i = 0; i < ntohs(mdns_hdr->qdcount); i++) {
		if (fc_mdns_check_query(mdns_hdr_start, &mdns_pos,
					&resp_info)) {
			return;
		}
	}
	if (!resp_info.need_ans) {
		LOG_DBG("Ignoring mDNS msg: no relevant query found.");
		return;
	}

	/* Check known-answer records */
	for (i = 0; i < ntohs(mdns_hdr->ancount); i++) {
		if (fc_mdns_check_known_ans(frame, mdns_hdr_start, &mdns_pos,
					    &resp_info)) {
			return;
		}
	}

	if (!resp_info.need_ans) {
		LOG_DBG("Ignoring mDNS msg: all queries have known-answers.");
		return;
	}

	/* Prepare mDNS response message */
	fr_process_mdns(frame, resp_info);
}

/* Frame Classifier: Prepare IPv6 neighbor advertisement packet */
static void fc_ipv6_nd(struct frame_classifier *frame,
		       struct net_eth_hdr *eth_hdr,
		       struct net_ipv6_hdr *ip_hdr, struct in6_addr *host_addr)
{
	struct net_eth_hdr *eth_resp_hdr;
	struct net_ipv6_hdr *ip_resp_hdr;
	struct net_icmp_hdr *icmp_resp_hdr;
	struct net_icmpv6_na_hdr *icmp_resp_na_hdr;
	struct net_icmpv6_nd_opt_hdr *icmp_resp_nd_opt_hdr;
	uint8_t *target_mac_addr;
	uint8_t flags;
	uint8_t resp_pkt[128];
	uint16_t chksum = 0;
	uint8_t *ptr;
	uint8_t *host_mac = np_mib.mac_addr;

	/* Copy the received neighbour solicitation packet to reuse packet
	 * length, traffic class, flow label, next header, hop limit,
	 * target address, etc.
	 */
	memcpy(resp_pkt, frame->pkt, frame->len);

	eth_resp_hdr = (struct net_eth_hdr *)resp_pkt;

	ip_resp_hdr = (struct net_ipv6_hdr *)(eth_resp_hdr + 1);

	icmp_resp_hdr = (struct net_icmp_hdr *)(ip_resp_hdr + 1);

	icmp_resp_na_hdr = (struct net_icmpv6_na_hdr *)(icmp_resp_hdr + 1);

	icmp_resp_nd_opt_hdr = (struct net_icmpv6_nd_opt_hdr *)(icmp_resp_na_hdr
								+ 1);

	target_mac_addr = (uint8_t *)(icmp_resp_nd_opt_hdr + 1);

	memcpy(&eth_resp_hdr->dst, &eth_hdr->src, NP_MAC_ADDR_BYTES);
	memcpy(&eth_resp_hdr->src, host_mac, NP_MAC_ADDR_BYTES);
	memcpy(target_mac_addr, host_mac, NP_MAC_ADDR_BYTES);

	/* Swap src and dst ip address */
	net_ipaddr_copy(&ip_resp_hdr->dst, &ip_hdr->src);
	net_ipaddr_copy(&ip_resp_hdr->src, host_addr);

	/* Logic from ipv6_nbr.c */
	icmp_resp_hdr->type = NET_ICMPV6_NA;
	icmp_resp_hdr->code = 0;
	icmp_resp_hdr->chksum = 0;
	flags = NET_ICMPV6_NA_FLAG_SOLICITED | NET_ICMPV6_NA_FLAG_OVERRIDE;
	icmp_resp_na_hdr->flags = flags;

	icmp_resp_nd_opt_hdr->type = NET_ICMPV6_ND_OPT_TLLAO;

	ptr = (uint8_t *)icmp_resp_hdr;
	chksum = netprox_calc_chksum(0, ptr, ip_resp_hdr->len);
	icmp_resp_hdr->chksum = ~htons(chksum);

	np_ctx.net_ctx->np_eth_tx_data(np_ctx.net_ctx->iface, resp_pkt,
				       frame->len);
	LOG_DBG("Send IPv6 Neighbor Advertisement message.");
}

/* Frame Classifier: Process IPv6 neighbor solicitation packet */
static void fc_process_nd(struct frame_classifier *frame)
{
	struct net_eth_hdr *eth_hdr;
	struct net_ipv6_hdr *ip_hdr;
	struct in6_addr *host_addr;
	struct net_icmp_hdr *icmp_hdr;
	uint8_t *start;
	uint8_t nexthdr;
	int i, j;
	uint8_t multicast_ipaddr[NETPROX_IPV6_MULTIC_IPADDR_SZ] = {
		0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x01, 0xFF
	};
	bool matched = 0;

	if ((fc_rs_ipv6 & NP_RL_CLS_ENABLE) != NP_RL_CLS_ENABLE) {
		LOG_DBG("Ignoring multicast IPv6 packet: handling decision = "
			"disable.");
		return;
	}

	eth_hdr = (struct net_eth_hdr *)frame->pkt;
	ip_hdr = (struct net_ipv6_hdr *)(eth_hdr + 1);

	/* Check the first 13 bytes of multicast IPv6 address */
	for (i = 0; i < NETPROX_IPV6_MULTIC_IPADDR_SZ; i++) {
		if (ip_hdr->dst.s6_addr[i] != multicast_ipaddr[i]) {
			LOG_DBG("Ignoring multicast IPv6 packet: not matching "
				"destination IPv6 address.");
			return;
		}
	}

	/* Compare the last 3 bytes of received multicast IPv6 address */
	for (i = 0; i < np_mib.ipv6_addr_sz; i++) {
		host_addr = (struct in6_addr *)np_mib.ipv6_addr[i];

		for (j = NETPROX_IPV6_MULTIC_IPADDR_SZ; j < NP_IPV6_ADDR_BYTES;
		     j++) {
			if (ip_hdr->dst.s6_addr[j] != host_addr->s6_addr[j]) {
				break;
			}
		}

		if (j == NP_IPV6_ADDR_BYTES) {
			matched = 1;
			break;
		}
	}

	if (!matched) {
		LOG_DBG("Ignoring multicast IPv6 packet: not matching "
			"destination IPv6 address.");
		return;
	}

	/* Check next header */
	nexthdr = ip_hdr->nexthdr;
	if (nexthdr != IPPROTO_ICMPV6) {
		LOG_DBG("Ignoring multicast IPv6 packet: not matching "
			"next-header %d.", nexthdr);
		return;
	}

	start = (uint8_t *)(ip_hdr + 1);
	icmp_hdr = (struct net_icmp_hdr *)start;

	/* Check ICMP header type */
	if (icmp_hdr->type == NET_ICMPV6_NS && icmp_hdr->code == 0) {
		fc_ipv6_nd(frame, eth_hdr, ip_hdr, host_addr);
	} else {
		LOG_DBG("Ignoring multicast IPv6 packet: not matching ICMP "
			"header type %d.", icmp_hdr->type);
		return;
	}
}

/* Check whether given IPv6 addr matches with any of the addr stored in MIB */
static int fc_check_ipv6_addr(struct in6_addr **host_addr,
			      struct net_ipv6_hdr *ip_hdr)
{
	int i;
	int ret = -1;

	/* Logic from ipv6.c net_ipv6_process_pkt() */
	for (i = 0; i < np_mib.ipv6_addr_sz; i++) {
		*host_addr = (struct in6_addr *)np_mib.ipv6_addr[i];

		if (net_ipv6_addr_cmp(*host_addr, &ip_hdr->dst)) {
			ret = 0;
			break;
		}
	}

	if (ret) {
		LOG_DBG("Ignoring received pkt: not matching destination "
			"IPv6 address.");
	}

	return ret;
}

/* Frame Classifier: Process IPv6 packet */
static void fc_ipv6(struct frame_classifier *frame)
{
	struct net_eth_hdr *eth_hdr;
	struct net_ipv6_hdr *ip_hdr;
	struct net_tcp_hdr *tcp_hdr;
	struct net_udp_hdr *udp_hdr;
	struct in6_addr *host_addr;
	struct net_icmp_hdr *icmp_hdr;
	uint8_t *ptr;
	uint8_t *start;
	uint8_t *host_mac;
	uint8_t nexthdr;
	uint16_t chksum = 0;
	int pkt_len;
	int len;
	bool process_hdr = true;

	eth_hdr = (struct net_eth_hdr *)frame->pkt;
	ip_hdr = (struct net_ipv6_hdr *)(eth_hdr + 1);
	host_mac = np_mib.mac_addr;

	/* The rest of the packet after IPv6 header */
	pkt_len = ntohs(ip_hdr->len);

	if (fc_check_ipv6_addr(&host_addr, ip_hdr)) {
		return;
	}

	/* No extension header */
	nexthdr = ip_hdr->nexthdr;
	start = (uint8_t *)(ip_hdr + 1);

	while (process_hdr) {
		switch (nexthdr) {
		case IPPROTO_TCP:
			tcp_hdr = (struct net_tcp_hdr *)start;
			fc_process_tcp(frame, tcp_hdr);
			process_hdr = false;
			break;

		case IPPROTO_ICMPV6:
			/* Logic from icmpv6.c */
			icmp_hdr = (struct net_icmp_hdr *)start;
			if (icmp_hdr->type == NET_ICMPV6_ECHO_REQUEST &&
			    icmp_hdr->code == 0) {
				/* Swap src and dst mac address */
				memcpy(&eth_hdr->dst, &eth_hdr->src,
				       NP_MAC_ADDR_BYTES);
				memcpy(&eth_hdr->src, host_mac,
				       NP_MAC_ADDR_BYTES);

				/* Swap src and dst ip address */
				net_ipaddr_copy(&ip_hdr->dst, &ip_hdr->src);
				net_ipaddr_copy(&ip_hdr->src, host_addr);

				icmp_hdr->type = NET_ICMPV6_ECHO_REPLY;
				icmp_hdr->code = 0;

				/* Chksum to 0 to calculate new chksum */
				icmp_hdr->chksum = 0;
				ptr = (uint8_t *)icmp_hdr;
				chksum = netprox_calc_chksum(0, ptr, pkt_len);
				icmp_hdr->chksum = ~htons(chksum);

				/* Add eth hdr + ipv6 main hdr len for tx */
				len = NETPROX_ETH_HDR_SIZE + NET_IPV6H_LEN
				      + pkt_len;
				ptr = (uint8_t *)eth_hdr;
				np_ctx.net_ctx->np_eth_tx_data(
					np_ctx.net_ctx->iface, ptr, len);
				LOG_DBG("Send ICMPv6 ping response pkt.");
			} else if (icmp_hdr->type == NET_ICMPV6_NS &&
				   icmp_hdr->code == 0) {
				fc_ipv6_nd(frame, eth_hdr, ip_hdr, host_addr);
			}
			process_hdr = false;
			break;

		case NET_IPV6_NEXTHDR_HBHO:
		case NET_IPV6_NEXTHDR_DESTO:
		case NET_IPV6_NEXTHDR_ROUTING:
			/* Extension header - Refer RFC 2460
			 * type(1st byte)
			 * length(2nd byte) in 8-octet units not including the
			 * first 8 octets.
			 */
			nexthdr = *start;
			len = *(start + 1);
			/* Point to next header */
			start += NETPROX_IPV6_EXT_HDR_LEN  + (len * 8);
			process_hdr = true;
			break;

		case NET_IPV6_NEXTHDR_FRAG:
			/* Fixed 8 bytes extension header */
			nexthdr = *start;
			/* Point to next header */
			start += NETPROX_IPV6_EXT_HDR_LEN;
			process_hdr = true;
			break;

		case NET_IPV6_NEXTHDR_NONE:
			LOG_DBG("Ignoring IPv6 pkt: next-header = NONE.");
			process_hdr = false;
			break;

		case IPPROTO_UDP:
			udp_hdr = (struct net_udp_hdr *)start;
			if (udp_hdr->dst_port ==
			    htons(NETPROX_SNMP_DEFAULT_UDP_PORT)) {
				fc_process_snmp(frame, udp_hdr, 1);
			} else {
				LOG_DBG("Ignoring IPv6 pkt: not matching "
					"destination UDP port.");
			}
			process_hdr = false;
			break;

		default:
			LOG_DBG("Ignoring IPv6 pkt: not matching "
				"next-header %d.", nexthdr);
			process_hdr = false;
			break;
		}
	}
}

/* Frame Classifier: Process IPv4 packet */
static void fc_ipv4(struct frame_classifier *frame)
{
	struct net_eth_hdr *eth_hdr;
	struct net_ipv4_hdr *ip_hdr;
	struct net_tcp_hdr *tcp_hdr;
	struct net_udp_hdr *udp_hdr;
	struct in_addr *host_addr;
	struct net_icmp_hdr *icmp_hdr;
	uint8_t *ptr;
	uint8_t *host_mac;
	uint16_t chksum = 0;
	int pkt_len;
	int len;

	eth_hdr = (struct net_eth_hdr *)frame->pkt;
	ip_hdr = (struct net_ipv4_hdr *)(eth_hdr + 1);
	host_mac = np_mib.mac_addr;
	host_addr = (struct in_addr *)np_mib.ipv4_addr;

	/* Total length of IP datagram */
	pkt_len = ntohs(ip_hdr->len);

	/* Logic from ipv4.c net_ipv4_process_pkt() */
	if (!net_ipv4_addr_cmp(host_addr, &ip_hdr->dst)) {
		LOG_DBG("Ignoring IPv6 pkt: not matching destination IPv4 "
			"Address.");
		return;
	}

	switch (ip_hdr->proto) {
	case IPPROTO_TCP:
		tcp_hdr = (struct net_tcp_hdr *)(ip_hdr + 1);
		fc_process_tcp(frame, tcp_hdr);
		break;

	case IPPROTO_ICMP:
		/* Logic from icmpv4.c */
		icmp_hdr = (struct net_icmp_hdr *)(ip_hdr + 1);
		if (icmp_hdr->type == NET_ICMPV4_ECHO_REQUEST &&
		    icmp_hdr->code == 0) {
			/* Swap src and dst mac address */
			memcpy(&eth_hdr->dst, &eth_hdr->src, NP_MAC_ADDR_BYTES);
			memcpy(&eth_hdr->src, host_mac, NP_MAC_ADDR_BYTES);

			/* Swap src and dst ip address */
			net_ipaddr_copy(&ip_hdr->dst, &ip_hdr->src);
			net_ipaddr_copy(&ip_hdr->src, host_addr);

			/* Change cheksum to 0 to calculate new cheksum */
			ip_hdr->chksum = 0;
			ptr = (uint8_t *)ip_hdr;
			chksum = netprox_calc_chksum(0, ptr, NET_IPV4H_LEN);
			ip_hdr->chksum = ~htons(chksum);

			icmp_hdr->type = NET_ICMPV4_ECHO_REPLY;
			icmp_hdr->code = 0;

			/* Change cheksum to 0 to calculate new cheksum */
			icmp_hdr->chksum = 0;
			ptr = (uint8_t *)icmp_hdr;
			/* Deduct IPv4 header length to obtain ICMP length */
			len = pkt_len - NET_IPV4H_LEN;
			chksum = netprox_calc_chksum(0, ptr, len);
			icmp_hdr->chksum = ~htons(chksum);

			/* Add eth header length + packet length for tx */
			len = NETPROX_ETH_HDR_SIZE + pkt_len;
			ptr = (uint8_t *)eth_hdr;
			np_ctx.net_ctx->np_eth_tx_data(np_ctx.net_ctx->iface,
						       ptr, len);
			LOG_DBG("Send ICMPv4 response pkt.");
		}
		break;

	case IPPROTO_UDP:
		udp_hdr = (struct net_udp_hdr *)(ip_hdr + 1);
		if (udp_hdr->dst_port == htons(NETPROX_SNMP_DEFAULT_UDP_PORT)) {
			fc_process_snmp(frame, udp_hdr, 0);
		} else {
			LOG_DBG("Ignoring IPv4 pkt: not matching destination "
				"UDP port.");
		}
		break;

	default:
		LOG_DBG("Ignoring IPv4 pkt: not matching protocol %02x.",
			ip_hdr->proto);
		break;
	}
}

/* Frame Classifier: Process any incoming packet */
static void fc_core(struct frame_classifier *frame)
{
	struct net_eth_hdr *eth_hdr;
	uint16_t type;
	uint8_t ipv6_maddr[NETPROX_IPV6_MADDR_SIZE] = { 0x33, 0x33, 0xff };

	eth_hdr = (struct net_eth_hdr *)frame->pkt;
	type = ntohs(eth_hdr->type);

	/* Pass the received packet to mDNS classifier if its destination MAC
	 * address match the Ipv4 or IPv6 mDNS multicast MAC address.
	 */
	if (memcmp(&eth_hdr->dst, mdns_db_hdr->mac4_maddr,
		   NP_MAC_ADDR_BYTES) == 0 && type == NET_ETH_PTYPE_IP) {
		fc_process_mdns(frame, NETPROX_IPV4_PACKET);
		return;
	}
	if (memcmp(&eth_hdr->dst, mdns_db_hdr->mac6_maddr,
		   NP_MAC_ADDR_BYTES) == 0 && type == NET_ETH_PTYPE_IPV6) {
		fc_process_mdns(frame, NETPROX_IPV6_PACKET);
		return;
	}

	/* Pass the received packet to neighbour discovery classifier if its
	 * destination MAC address match IPv6 multicast MAC address.
	 */
	if (memcmp(&eth_hdr->dst, ipv6_maddr, NETPROX_IPV6_MADDR_SIZE) == 0 &&
	    type == NET_ETH_PTYPE_IPV6) {
		fc_process_nd(frame);
		return;
	}

	/* Ignore the received packet if its destination MAC address do not
	 * match the Network Proxy MAC address.
	 */
	if (memcmp(&eth_hdr->dst, np_mib.mac_addr, NP_MAC_ADDR_BYTES)) {
		LOG_DBG("Ignoring received pkt: not matching destination "
			"MAC address.");
		return;
	}

	/* TODO: software checksum checking */

	/* Logic from ethernet.c ethernet_recv() */
	switch (type) {
	case NET_ETH_PTYPE_IPV6:
		if (((fc_rs_ipv6 & NP_RL_CLS_ENABLE) == NP_RL_CLS_ENABLE)) {
			fc_ipv6(frame);
		} else {
			LOG_DBG("Ignoring IPv6 pkt: handling decision = "
				"disable.");
		}
		break;

	case NET_ETH_PTYPE_IP:
		if (((fc_rs_ipv4 & NP_RL_CLS_ENABLE) == NP_RL_CLS_ENABLE)) {
			fc_ipv4(frame);
		} else {
			LOG_DBG("Ignoring IPv4 pkt: handling decision = "
				"disable.");
		}
		break;

	default:
		LOG_DBG("Ignoring received pkt: not matching ETH Type %02x.",
			type);
		break;
	}
}

/* Store incoming packets for Frame Classifier */
void frm_cls_add_pkt(uint8_t *rxbuf, uint16_t len)
{
	int ptr = frm_cls.pkt_wr_ptr;

	/* Wait for packet buffer ring slot available */
	while (((ptr + 1) & PKT_BUF_SZ_MASK) == frm_cls.pkt_rd_ptr) {
		k_sleep(K_MSEC(5)); /* 5ms polling interval */
	}

	/* Fill packet into packet buffer ring */
	memcpy(frm_cls.pkt_buf[ptr], rxbuf, len);
	frm_cls.pkt_len_buf[ptr] = len;
	frm_cls.pkt_wr_ptr = (frm_cls.pkt_wr_ptr + 1) & PKT_BUF_SZ_MASK;

	k_work_submit_to_queue(&frm_cls.netprox_cls_wq,
			       &frm_cls.netprox_cls_work);
}

/* Frame Classifier: Process incoming packets one by one */
static void frm_cls_core(struct k_work *item)
{
	struct frame_classifier *frame =
		CONTAINER_OF(item, struct frame_classifier, netprox_cls_work);

	np_ctx.net_ctx->np_cls_in_process = true;

	/* Loop until packet buffer ring buffer empty */
	for (frame->pkt = frame->pkt_buf[frame->pkt_rd_ptr],
	     frame->len = frame->pkt_len_buf[frame->pkt_rd_ptr];
	     frame->pkt_rd_ptr != frame->pkt_wr_ptr;
	     frame->pkt_rd_ptr = (frame->pkt_rd_ptr + 1) & PKT_BUF_SZ_MASK,
	     frame->pkt = frame->pkt_buf[frame->pkt_rd_ptr],
	     frame->len = frame->pkt_len_buf[frame->pkt_rd_ptr]) {
		if (np_ctx.add_a2h_pkt) {
			netprox_add_a2h_packet(frame->pkt, frame->len);
			continue;
		}
		fc_core(frame);
	}

	np_ctx.net_ctx->np_cls_in_process = false;
}

static void netprox_shm_data_copy(void *dest, const void *src, size_t size)
{
	/* TODO: use DMA engine for shmem copy */
	memcpy(dest, src, size);
}

/* Get Network Proxy rule information */
static void netprox_rule_info_get(struct np_rules *rule, uint8_t **info_dest,
				  int *info_size)
{
	/* Initialize return value for fail cases */
	*info_dest = NULL;
	*info_size = 0;

	switch (rule->group) {
	case NP_RL_G_CLS:
		*info_size = FC_RS_SIZE;
		if (rule->type == NP_RL_T_IPV4) {
			*info_dest = (uint8_t *)&fc_rs_ipv4;
		} else if (rule->type == NP_RL_T_IPV6) {
			*info_dest = (uint8_t *)&fc_rs_ipv6;
		} else if (rule->type == NP_RL_T_TCP_WAKE_PORT) {
			*info_dest = (uint8_t *)&fc_rs_tcp_wake_port;
		} else if (rule->type == NP_RL_T_SNMP) {
			*info_dest = (uint8_t *)&fc_rs_snmp;
		} else if (rule->type == NP_RL_T_MDNS) {
			*info_dest = (uint8_t *)&fc_rs_mdns;
		}
		break;
	case NP_RL_G_MIB:
		if (rule->type == NP_RL_T_MAC_ADDR) {
			*info_dest = (uint8_t *)&np_mib.mac_addr;
			*info_size = sizeof(np_mib.mac_addr);
		} else if (rule->type == NP_RL_T_IPV4) {
			*info_dest = (uint8_t *)&np_mib.ipv4_addr;
			*info_size = sizeof(np_mib.ipv4_addr);
		} else if (rule->type == NP_RL_T_IPV6) {
			*info_dest = (uint8_t *)&np_mib.ipv6_addr;
			*info_size = sizeof(np_mib.ipv6_addr);
		} else if (rule->type == NP_RL_T_TCP_WAKE_PORT) {
			*info_dest = (uint8_t *)&np_mib.tcp_port;
			*info_size = sizeof(np_mib.tcp_port);
		} else if (rule->type == NP_RL_T_SNMP_COMMUNITY_STR) {
			*info_dest =
				(uint8_t *)&snmp_oid_tree_info.community_str;
			/* Last character is always NULL terminated */
			*info_size = NETPROX_SNMP_MAX_COMMUNITY_STR_SZ - 1;
		} else if (rule->type == NP_RL_T_IPV4_SUBNET) {
			*info_dest = (uint8_t *)&np_mib.ipv4_subnet;
			*info_size = sizeof(np_mib.ipv4_subnet);
		}
		break;
	}
}

/* Read Network Proxy Frame Classifier's rules */
static void netprox_cls_read_rules(struct np_rules *rule)
{
	struct np_rules *pr_rule;
	uint8_t *prule_val;
	int r_rule_size;
	int r_info_size;
	uint8_t *pinfo_src = NULL;
	int info_size = 0;

	netprox_rule_info_get(rule, &pinfo_src, &info_size);

	if (!pinfo_src || rule->offset >= info_size) {
		/* Not supported */
		LOG_ERR("Invalid read rule condition.");
		return;
	}

	/* Make sure info read size does not exceed info_size */
	if (info_size < (rule->size + rule->offset)) {
		r_info_size = info_size - rule->offset;
	} else {
		r_info_size = rule->size;
	}

	r_rule_size = sizeof(struct np_rules) + r_info_size;
	if (r_rule_size > NP_IPC_PYLD_MAX) {
		LOG_ERR("Read rule size more than IPC supported.");
		return;
	}

	pr_rule = (struct np_rules *)k_malloc(r_rule_size);
	if (!pr_rule) {
		LOG_ERR("Read rule no memory.");
		return;
	}

	/* Copy read rule info and send back for host to compare */
	memcpy(pr_rule, rule, sizeof(struct np_rules));

	/* Point to value[0] of struct np_rules */
	prule_val = (uint8_t *)(pr_rule + 1);

	/* Offset and copy the content */
	pinfo_src += rule->offset;
	memcpy(prule_val, pinfo_src, r_info_size);

	netprox_send_message(NP_A2H_CMD_READ_CLS_RULE_RESULT,
			     (uint8_t *)pr_rule,
			     r_rule_size);

	k_free(pr_rule);
}

/* Write Network Proxy Frame Classifier's rules */
static void netprox_cls_write_rules(struct np_rules *rule)
{
	uint8_t *pinfo_dest = NULL;
	int info_size = 0;
	unsigned int offset = rule->offset;
	struct snmp_oid_tree_info *p_oid_info = &snmp_oid_tree_info;

	netprox_rule_info_get(rule, &pinfo_dest, &info_size);

	if (!pinfo_dest || (offset + rule->size) > info_size) {
		/* Not supported */
		LOG_ERR("Invalid write rule condition.");
		return;
	}

	/* Clear buffer of rule info */
	memset((void *)pinfo_dest, 0, info_size);

	/* Offset and copy the content */
	pinfo_dest += rule->offset;
	memcpy(pinfo_dest, rule->value, rule->size);

	/* More action after write rule done */
	switch (rule->group) {
	case NP_RL_G_MIB:
		if (rule->type == NP_RL_T_IPV4) {
			/* Inform host agent is ready after setup ipv4 mib */
			netprox_send_message(NP_A2H_CMD_AGENT_READY, NULL, 0);
		} else if (rule->type == NP_RL_T_IPV6) {
			np_mib.ipv6_addr_sz = (offset + rule->size) /
					      sizeof(np_mib.ipv6_addr[0]);
		} else if (rule->type == NP_RL_T_TCP_WAKE_PORT) {
			np_mib.tcp_port_sz = (offset + rule->size) /
					     sizeof(np_mib.tcp_port[0]);
		} else if (rule->type == NP_RL_T_SNMP_COMMUNITY_STR) {
			/* Terminate community string with NULL character */
			*(pinfo_dest + rule->size) = '\0';
			/* NULL character is for display termination in Host.
			 * In Agent, we don't use the NULL character for
			 * comparison. Therefore, we only store the length of
			 * the community string.
			 */
			p_oid_info->community_str_sz = rule->size;
		}
		break;
	}
}

/* Write to shared memory region */
static void netprox_write_shm_data(struct np_rules *rule)
{
	struct np_shm_info info;
	uint8_t *ptr;
	uint8_t *cache_root;
	unsigned int *cache_sz;
	unsigned int cache_max_sz;

	memcpy(&info, rule->value, sizeof(struct np_shm_info));
	k_mutex_lock(&shmem_mutex, K_FOREVER);

	switch (rule->type) {
	case NP_RL_T_SNMP_WRITE_OID_TREE:
		cache_max_sz = NETPROX_SNMP_MAX_OID_TREE_SZ;
		cache_sz = &snmp_oid_tree_info.oid_tree_sz;
		cache_root = snmp_oid_tree_info.oid_tree;
		break;

	case NP_RL_T_MDNS_WRITE_RR:
		cache_max_sz = NETPROX_MDNS_MAX_RR_SZ;
		cache_sz = &mdns_db.sz;
		cache_root = mdns_db.data;
		break;

	default:
		LOG_ERR("Not supported rule.");
		goto finish;
	}

	/* Check whether size of pre-allocated cache is sufficient to store
	 * given buffer
	 */
	if (info.total_size > cache_max_sz ||
	    (info.offset + info.size) > cache_max_sz) {
		LOG_DBG("Buffer size > supported cache size.");
		goto finish;
	}

	/* Total shared memory size stated by every memory passing should be
	 * same as first time passing
	 */
	if (info.offset) {
		if (*cache_sz != info.total_size) {
			LOG_ERR("Inconsistent total shared memory size.");
			goto finish;
		}
	} else {
		*cache_sz = info.total_size;
	}

	if (info.size <= np_ctx.shmem_info.np_shmem_sz) {
		ptr = (uint8_t *)np_ctx.shmem_info.np_shmem_addr;
		dcache_invalidate((uint32_t)ptr, info.size);
		netprox_shm_data_copy(cache_root + info.offset, ptr, info.size);
		netprox_send_message(NP_A2H_CMD_SHM_DATA_COMPLETE, NULL, 0);
	} else {
		LOG_DBG("Buffer size bigger than supported shared mem size.");
	}

finish:
	k_mutex_unlock(&shmem_mutex);
}

/* Read from shared memory region */
static void netprox_read_shm_data(struct np_rules *rule)
{
	struct np_shm_info *info;
	struct np_rules *send_rule;
	int send_size;
	uint8_t *ptr;
	uint8_t *cache_root;
	unsigned int cache_sz;

	k_mutex_lock(&shmem_mutex, K_FOREVER);

	switch (rule->type) {
	case NP_RL_T_SNMP_READ_OID_TREE:
		cache_sz = snmp_oid_tree_info.oid_tree_sz;
		cache_root = snmp_oid_tree_info.oid_tree;
		break;

	case NP_RL_T_MDNS_READ_RR:
		cache_sz = mdns_db.sz;
		cache_root = mdns_db.data;
		break;

	default:
		LOG_ERR("Not supported rule.");
		goto finish;
	}

	send_size = sizeof(struct np_rules) + sizeof(struct np_shm_info);
	send_rule = (struct np_rules *)k_malloc(send_size);
	if (!send_rule) {
		LOG_ERR("Send rule no memory.");
		goto finish;
	}

	/* Copy read rule info and send back for host to compare */
	memcpy(send_rule, rule, send_size);
	info = (struct np_shm_info *)(send_rule + 1);

	if (info->offset >= cache_sz) {
		LOG_ERR("Offset requested >= available cache size.");
		goto free_rule;
	}

	info->total_size = cache_sz;

	if (cache_sz - info->offset >= np_ctx.shmem_info.np_shmem_sz) {
		info->size = np_ctx.shmem_info.np_shmem_sz;
	} else {
		info->size = cache_sz - info->offset;
	}

	ptr = (uint8_t *)np_ctx.shmem_info.np_shmem_addr;
	netprox_shm_data_copy(ptr, cache_root + info->offset, info->size);
	dcache_clean((uint32_t)ptr, info->size);
	netprox_send_message(NP_A2H_CMD_SHM_DATA_COMPLETE, (uint8_t *)send_rule,
			     send_size);

free_rule:
	k_free(send_rule);

finish:
	k_mutex_unlock(&shmem_mutex);
}

/* Process received IPC message */
static void netprox_process_message(uint8_t *buf)
{
	struct np_ipc_hdr *rxipc_hdr = (struct np_ipc_hdr *)buf;
	struct np_ipc_hdr *txipc_hdr;
	struct np_rules *rules;
	uint8_t *ptr;

	mrd_t m = { 0 };

	txipc_hdr = (struct np_ipc_hdr *)np_ipc_tx_buf;
	ptr = np_ipc_tx_buf;
	txipc_hdr->command = NP_IS_RESPONSE;
	txipc_hdr->status = 0;
	txipc_hdr->size = 0;

	LOG_DBG("Received IPC cmd: %d.", rxipc_hdr->command);
	switch (rxipc_hdr->command) {
	case NP_H2A_CMD_NETDEV_READY:
		m.buf = np_ipc_tx_buf;
		m.len = sizeof(*txipc_hdr);
		heci_send(netprox_conn_id, &m);

		struct np_agent_info info;

		info.major = NETPROX_AGENT_MAJOR;
		info.minor = NETPROX_AGENT_MINOR;
		info.revision = NETPROX_AGENT_REVISION;
		info.max_cls_rules = NETPROX_AGENT_MAX_CLS_RULES;
		info.max_resp_rules = NETPROX_AGENT_MAX_RESP_RULES;

		netprox_send_message(NP_A2H_CMD_AGENT_INFO, (uint8_t *)&info,
				     sizeof(info));
		break;

	case NP_H2A_CMD_WRITE_CLS_RULE:
		m.buf = np_ipc_tx_buf;
		m.len = sizeof(*txipc_hdr);
		rules = (struct np_rules *)(rxipc_hdr + 1);
		heci_send(netprox_conn_id, &m);
		netprox_cls_write_rules(rules);
		break;

	case NP_H2A_CMD_READ_CLS_RULE:
		m.buf = np_ipc_tx_buf;
		m.len = sizeof(*txipc_hdr);
		rules = (struct np_rules *)(rxipc_hdr + 1);
		heci_send(netprox_conn_id, &m);
		netprox_cls_read_rules(rules);
		break;

	case NP_H2A_CMD_PROXY_EXIT:
		m.buf = np_ipc_tx_buf;
		m.len = sizeof(*txipc_hdr);
		/* Ack host received exit network proxy mode */
		heci_send(netprox_conn_id, &m);
		/* For continue add A2H packet while host is resume */
		np_ctx.add_a2h_pkt = true;
		np_ctx.net_ctx->np_exit(np_ctx.net_ctx->iface);

		netprox_send_message(NP_A2H_CMD_HOST_IS_EXITED, NULL, 0);
		/* Done proxy exit flow, reset state */
		np_ctx.a2h_rxbuf.a2h_total_packets = 0;
		np_ctx.a2h_rxbuf.a2h_total_size = A2H_HEADER_SIZE;
		np_ctx.add_a2h_pkt = false;
		np_ctx.waked_host = false;
		break;

	case NP_H2A_CMD_WRITE_SHM_DATA:
		m.buf = np_ipc_tx_buf;
		m.len = sizeof(*txipc_hdr);
		rules = (struct np_rules *)(rxipc_hdr + 1);
		heci_send(netprox_conn_id, &m);
		netprox_write_shm_data(rules);
		break;

	case NP_H2A_CMD_READ_SHM_DATA:
		m.buf = np_ipc_tx_buf;
		m.len = sizeof(*txipc_hdr);
		rules = (struct np_rules *)(rxipc_hdr + 1);
		heci_send(netprox_conn_id, &m);
		netprox_read_shm_data(rules);
		break;

	default:
		LOG_ERR("Get unknown command %d.", rxipc_hdr->command);
		break;
	}
}

static void netprox_event_callback(uint32_t event, void *param)
{
	netprox_event = event;
	k_sem_give(&netprox_event_sem);
}

/* Handle HECI event */
static void netprox_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_DBG("%s enter.", __func__);

	while (true) {
		k_sem_take(&netprox_event_sem, K_FOREVER);

		LOG_DBG("Netprox new heci event %u.", netprox_event);

		switch (netprox_event) {
		case HECI_EVENT_NEW_MSG:
			if (np_ipc_rx_msg.msg_lock != MSG_LOCKED) {
				LOG_ERR("Invalid heci message.");
				break;
			}

			if (np_ipc_rx_msg.type == HECI_CONNECT) {
				netprox_conn_id = np_ipc_rx_msg.connection_id;
				LOG_DBG("New conn: %u.", netprox_conn_id);
			} else if (np_ipc_rx_msg.type == HECI_REQUEST) {
				netprox_process_message(np_ipc_rx_msg.buffer);
			}

			/*
			 * Send flow control after finishing one message,
			 * allow host to send new request
			 */
			heci_send_flow_control(netprox_conn_id);
			break;

		case HECI_EVENT_DISCONN:
			LOG_DBG("Disconnect request conn %d.",
				netprox_conn_id);
			heci_complete_disconnect(netprox_conn_id);
			break;

		default:
			LOG_ERR("Wrong heci event %u.", netprox_event);
			break;
		}
	}
}

void netprox_register_a2h_info(uint32_t a2h_addr, uint32_t a2h_max)
{
	np_ctx.a2h_rxbuf.a2h_addr = a2h_addr;
	np_ctx.a2h_rxbuf.a2h_max = a2h_max;
}

void netprox_register_shmem_info(uint32_t np_shmem_addr, uint32_t np_shmem_sz)
{
	np_ctx.shmem_info.np_shmem_addr = np_shmem_addr;
	np_ctx.shmem_info.np_shmem_sz = np_shmem_sz;
}

void netprox_register(struct netprox_net *net_ctx)
{
	np_ctx.net_ctx = net_ctx;
}

void netprox_configure_arp_offload(void)
{
	struct netprox_net *net_ctx = np_ctx.net_ctx;

	if (fc_rs_ipv4 & NP_RL_CLS_ENABLE) {
		net_ctx->arp_offload(net_ctx->iface, true,
				     ntohl(*(uint32_t *)np_mib.ipv4_addr));
	}
}

void netprox_get_ipv4_addr(uint32_t *ipv4_addr)
{
	*ipv4_addr = *(uint32_t *)np_mib.ipv4_addr;
}

void netprox_get_mac_addr(struct net_eth_addr *mac_addr)
{
	*mac_addr = *(struct net_eth_addr *)np_mib.mac_addr;
}

static int netprox_init(const struct device *arg)
{
	ARG_UNUSED(arg);

	int ret;
	heci_client_t netprox_client = {
		.protocol_id = HECI_CLIENT_NETPROX_GUID,
		.max_msg_size = NP_IPC_MSG_MAX,
		.protocol_ver = 1,
		.max_n_of_connections = 1,
		.dma_header_length = 0,
		.dma_enabled = 0,
		.rx_buffer_len = NP_IPC_MSG_MAX,
		.event_cb = netprox_event_callback,
	};

	netprox_client.rx_msg = &np_ipc_rx_msg;
	netprox_client.rx_msg->buffer = np_ipc_rx_buf;

	ret = heci_register(&netprox_client);
	if (ret) {
		LOG_ERR("Failed to register netprox client %d.", ret);
		return ret;
	}

	k_thread_create(&netprox_thread, netprox_stack, NETPROX_STACK_SIZE,
			netprox_task, NULL, NULL, NULL,
			K_PRIO_PREEMPT(11), 0, K_NO_WAIT);

	k_work_queue_start(&frm_cls.netprox_cls_wq, netprox_cls_wq_stack,
			   K_THREAD_STACK_SIZEOF(netprox_cls_wq_stack),
			   K_LOWEST_APPLICATION_THREAD_PRIO, NULL);

	frm_cls.pkt_rd_ptr = 0;
	frm_cls.pkt_wr_ptr = 0;
	k_work_init(&frm_cls.netprox_cls_work, frm_cls_core);

	return 0;
}

/* TODO: Use SYS_INIT or call from eth_dwc_eqos driver ?*/
SYS_INIT(netprox_init, POST_KERNEL, 80);
