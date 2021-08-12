/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_ETHERNET_ETH_DWC_EQOS_NETPROX_PROTO_H_
#define DRIVERS_ETHERNET_ETH_DWC_EQOS_NETPROX_PROTO_H_

#include "eth_dwc_eqos_v5_netprox.h"

/* Cache alignment */
#define NETPROX_DCACHE_ALIGNMENT		32

#define NETPROX_AGENT_MAJOR			1
#define NETPROX_AGENT_MINOR			0
#define NETPROX_AGENT_REVISION			0
#define NETPROX_AGENT_MAX_CLS_RULES		20
#define NETPROX_AGENT_MAX_RESP_RULES		20
#define NETPROX_AGENT_MAX_TCP_WAKE_PORT		16

#define NETPROX_ETH_HDR_SIZE		sizeof(struct net_eth_hdr)
#define NETPROX_IPV6_EXT_HDR_LEN	8
#define NETPROX_ICMPV6_NA_HDR_LEN	sizeof(struct net_icmpv6_na_hdr)

#define NETPROX_IPV6_PACKET		1
#define NETPROX_IPV4_PACKET		0
#define NETPROX_IPV6_MADDR_SIZE		3
#define NETPROX_IPV6_MULTIC_IPADDR_SZ	13

#define NETPROX_UDP_PORT_BYTES		2

#define HECI_CLIENT_NETPROX_GUID	{ 0x1586a9d4,			       \
					  0xae85, 0x4f4c,		       \
					  { 0x90, 0x72, 0x57, 0x56,	       \
					    0x0b, 0xd5, 0x27, 0x1e } }

/* Stack size for Network Proxy task */
#define NETPROX_STACK_SIZE		1600
#define NETPROX_CLS_WQ_STACK_SIZE	4096

#define A2H_HEADER_SIZE			sizeof(struct np_a2h_pool_header)
#define A2H_PKT_HEADER_SIZE		sizeof(struct np_a2h_packet_header)
#define A2H_NEW_PKT_SIZE		(A2H_PKT_HEADER_SIZE + NP_A2H_PKT_MAX)

enum cls_type {
	CLS_PERFECT_MATCH,
	CLS_MASK_MATCH,
};

/* TODO: frame modifier k_work */
#define PKT_BUF_SZ			32	/* Value must be power of 2 */
#define PKT_BUF_SZ_MASK			(PKT_BUF_SZ - 1)
#define PKT_MAX_SZ			(1504 + NETPROX_ETH_HDR_SIZE)

struct frame_classifier {
	struct k_work_q netprox_cls_wq;
	struct k_work netprox_cls_work;
	uint8_t pkt_buf[PKT_BUF_SZ][PKT_MAX_SZ];
	uint16_t pkt_len_buf[PKT_BUF_SZ];
	int pkt_rd_ptr;
	int pkt_wr_ptr;
	uint8_t *pkt;
	uint16_t len;
};

/* SNMP v1 & v2c Message Format (TLV Format):-
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
/* Type of SNMP version */
#define NETPROX_SNMP_VERSION_V1			0
#define NETPROX_SNMP_VERSION_V2C		1

/* Type of SNMP PDU */
#define NETPROX_SNMP_PDU_GET			0xA0
#define NETPROX_SNMP_PDU_GET_NEXT		0xA1
#define NETPROX_SNMP_PDU_RESPONSE		0xA2
#define NETPROX_SNMP_PDU_SET			0xA3

/* Type of SNMP Message Error */
#define NETPROX_SNMP_ERROR_NOSUCHNAME		0x02

/* Type of OID data */
#define NETPROX_SNMP_TYPE_INTEGER		0x02
#define NETPROX_SNMP_TYPE_NULL			0x05
#define NETPROX_SNMP_TYPE_OID			0x06
#define NETPROX_SNMP_TYPE_SEQUENCE		0x30

/* Common SNMP protocol default definitions */
#define NETPROX_SNMP_DEFAULT_UDP_PORT		161
#define NETPROX_SNMP_MAX_OID_DEPTH		16

/* SNMP response info extracted from received SNMP message and OID tree */
struct snmp_response_info {
	uint8_t oid[NETPROX_SNMP_MAX_OID_DEPTH];
	uint8_t oid_total_depth;	/* Total depth of OID */
	uint16_t msg_len;		/* Length of SNMP Message */
	uint16_t pdu_len;		/* Length of SNMP PDU */
	uint16_t vbl_len;		/* Length of Varbind List */
	uint16_t vb_len;		/* Length of Varbind */
	uint16_t val_len;		/* Length of Value */
	uint8_t val_type;		/* Type of value */
	uint8_t *val;			/* Pointer to value */
	bool is_ipv6;
	bool get_next;
};

#define SNMP_NEXT_FIELD_OFFSET(offset)					       \
	(offset + 2)
#define SNMP_FIELD(offset)						       \
	(*(snmp_hdr + offset))
#define SNMP_FIELD_TYPE(offset)						       \
	(*(snmp_hdr + offset))
#define SNMP_FIELD_LEN(offset)						       \
	(*(snmp_hdr + offset + 1))
#define SNMP_FIELD_LEN_FIRST_OCTET(offset)				       \
	(*(snmp_hdr + offset + 2))
#define SNMP_FIELD_LEN_SECOND_OCTET(offset)				       \
	(*(snmp_hdr + offset + 3))
#define SNMP_FIELD_VALUE_BYTE(offset)					       \
	(*(snmp_hdr + offset + 2))
#define SNMP_FIELD_LONG_FORM_VALUE_BYTE(offset)				       \
	(*(snmp_hdr + offset + 4))
#define SNMP_FIELD_VALUE_PTR(offset)					       \
	(snmp_hdr + offset + 2)
#define SNMP_GET_UPPER_BYTE_LEN(len)					       \
	((len) >> 8)

/* Structure for SNMP message field offset
 *
 * This structure is used to calculate offset of respective field in the
 * received SNMP message following the above SNMP message format.
 */
struct snmp_msg_field_offset {
	uint8_t ver;		/* Version */
	uint8_t community_str;	/* Community String */
	uint8_t pdu;		/* SNMP PDU */
	uint8_t id;		/* Request ID */
	uint8_t err;		/* Error */
	uint8_t err_idx;	/* Error Index */
	uint8_t vbl;		/* Varbind List */
	uint8_t vb;		/* Varbind */
	uint8_t oid;		/* Object Identifier */
	uint8_t val;		/* Value */
};

/* In the long form data (length bigger than 127 bytes), the length octets will
 * consist of an initial octet and one or more subsequent octets.
 *
 * The initial octet shall be encoded as follows:
 * a) bits 0 - 6 shall encode the number of subsequent octets
 * b) bit 7 shall be one
 *
 * The subsequent octets will record the total length of data.
 */
#define NETPROX_SNMP_LONG_FORM_TWO_OCTET	(BIT(7) | BIT(1))
#define NETPROX_SNMP_ONE_SUB_OCTET		1
#define NETPROX_SNMP_TWO_SUB_OCTET		2
#define NETPROX_SNMP_FOUR_SUB_OCTET		4
#define NETPROX_SNMP_SIX_SUB_OCTET		6
#define NETPROX_SNMP_EIGHT_SUB_OCTET		8

/* Structure for SNMP OID tree information
 *
 * Please make sure the value of NETPROX_SNMP_MAX_COMMUNITY_STR_SZ and
 * NETPROX_SNMP_MAX_OID_TREE_SZ is always word aligned.
 *
 * The SNMP OID tree is stored in oid_tree[NETPROX_SNMP_MAX_OID_TREE_SZ]
 * following the order of three regions each containing many instances of
 * struct oid_node, struct oid_data_info, and uint8_t data as shown below.
 *
 * OID Tree Diagram :-
 * ===================
 *  +---------------+---------------+---------------+
 *  |                                               |
 *  /            Region (struct oid_node)           /
 *  |                                               |
 *  +---------------+---------------+---------------+
 *  |                                               |
 *  /         Region (struct oid_data_info)         /
 *  |                                               |
 *  +---------------+---------------+---------------+
 *  |                                               |
 *  /              Region (uint8_t data)            /
 *  |                                               |
 *  +---------------+---------------+---------------+
 */
#define NETPROX_SNMP_MAX_COMMUNITY_STR_SZ	32
/* OID tree size: 380 kilobytes */
#define NETPROX_SNMP_MAX_OID_TREE_SZ		389120

struct snmp_oid_tree_info {
	uint8_t oid_tree[NETPROX_SNMP_MAX_OID_TREE_SZ];
	uint8_t community_str[NETPROX_SNMP_MAX_COMMUNITY_STR_SZ];
	unsigned int oid_tree_sz;
	unsigned int community_str_sz;
};

#define SNMP_OID_NODE_PTR(offset)					       \
	((struct oid_node *)&snmp_oid_tree_info.oid_tree[offset])
#define SNMP_OID_DATA_INFO_PTR(offset)					       \
	((struct oid_data_info *)&snmp_oid_tree_info.oid_tree[offset])
#define SNMP_OID_DATA_PTR(offset)					       \
	((uint8_t *)&snmp_oid_tree_info.oid_tree[offset])
/* Used to decide whether oid_data_info.data contains OID data or
 * offset to uint8_t OID data.
 */
#define SNMP_OID_DATA_SZ			4

/* Structure for OID node */
struct oid_node {
	uint8_t id;		/* Value of OID node */
	uint8_t depth;		/* Depth of OID node */
	uint8_t has_info;	/* If true, 'child' is an offset to struct
				 * oid_data_info.
				 */
	uint8_t pad;
	uint32_t parent;	/* Offset to previous OID node */
	uint32_t next;		/* Offset to neighbor OID node (same level) */
	uint32_t child;		/* Offset to next level of OID node or
				 * oid_data_info
				 */
};

/* Structure for OID data (value) information */
struct oid_data_info {
	uint8_t data_len;	/* Length of OID data in bytes.
				 * If data_len > SNMP_OID_DATA_SZ, then 'data'
				 * is an offset to uint8_t OID data. Else,
				 * 'data' is just a value and its size depends
				 * on data_len.
				 */
	uint8_t data_type;	/* Type of OID data */
	uint16_t request_count;	/* Total number of this OID being requested */
	uint32_t data;		/* OID data or offset to uint8_t OID data */
};

/* Common mDNS protocol default definitions */
#define NETPROX_MDNS_DEFAULT_UDP_PORT		5353
#define NETRPOX_MDNS_DEFAULT_IPV4_TTL		255

/* Limit mDNS PTR query response wait time to 63 ms */
#define NETPROX_MDNS_PTR_WAIT_TIME_MASK		0x0000003F

/* mDNS Message Header Section Format :-
 * =====================================
 *  00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 * |                      ID                       |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 * |QR|   Opcode  |AA|TC|RD|RA|    Z   |   RCODE   |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 * |                    QDCOUNT                    |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 * |                    ANCOUNT                    |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 * |                    NSCOUNT                    |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 * |                    ARCOUNT                    |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 */
#define NETPROX_MDNS_HDR_FLAGS_QR	BIT(15)	/* 0: query & 1: response */
#define NETPROX_MDNS_HDR_FLAGS_AA	BIT(10)	/* 1: authoritative answer */
#define NETPROX_MDNS_HDR_FLAGS_TC	BIT(9)	/* 1: truncated */

/* mDNS message header */
struct mdns_hdr {
	uint16_t id;			/* Query identifier */
	uint16_t flags;			/* Flags */
	uint16_t qdcount;		/* Query domain count */
	uint16_t ancount;		/* Answer count */
	uint16_t nscount;		/* Name server (authority) count */
	uint16_t arcount;		/* Additional records count */
};

/* mDNS Message Question Section Format :-
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
#define NETPROX_MDNS_QTYPE_A		0x01	/* IPv4 Host Address */
#define NETPROX_MDNS_QTYPE_PTR		0x0C	/* Domain name pointer */
#define NETPROX_MDNS_QTYPE_TXT		0x10	/* Text strings */
#define NETPROX_MDNS_QTYPE_AAAA		0x1C	/* IPv6 Host Address */
#define NETPROX_MDNS_QTYPE_SRV		0x21	/* Service */

#define NETPROX_MDNS_QCLASS_QU		BIT(15)	/* 1: unicast & 0: multicast */
#define NETPROX_MDNS_QCLASS_IN		0x01	/* Internet */

#define NETPROX_MDNS_MAX_NAME_SZ	256

/* Struct for keeping track of content of mDNS Question */
struct mdns_question_info {
	uint8_t name[NETPROX_MDNS_MAX_NAME_SZ];	/* Query name */
	uint16_t name_len;			/* Length of query name
						 * including zero-length NULL
						 */
	uint16_t type;				/* Query type */
	uint16_t class;				/* Query class */
};

/* mDNS Message Resource Record (RR) Format :-
 * ===========================================
 * Note: For Answer, Authoritative, and Additional Records.
 *
 *  00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 * |                                               |
 * /                     NAME                      /
 * |                                               |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+  --
 * |                     TYPE                      |   |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+   |
 * |CF|                  CLASS                     |   |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+   |
 * |                      TTL                      |   }  metadata
 * |                                               |   |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+   |
 * |                    RDLENGTH                   |   |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+  --
 * |                                               |
 * /                     RDATA                     /
 * |                                               |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 *
 * Special Note:
 * For mDNS query and known-answer processing efficiency, we declare metadata
 * as collection of TYPE, CLASS, TTL, and RDLENGTH.
 */
#define NETPROX_MDNS_RR_CLASS_CF	BIT(15)	/* 1: cache flush */

#define NETPROX_MDNS_TYPE_SZ		2
#define NETPROX_MDNS_CLASS_SZ		2
#define NETPROX_MDNS_TTL_SZ		4
#define NETPROX_MDNS_RDLENGTH_SZ	2
#define NETPROX_MDNS_METADATA_SZ	10

/* The largest RDATA content is typically used for TXT records, for now, we
 * limit the size of RDATA to 1k bytes.
 */
#define NETPROX_MDNS_MAX_RDATA_SZ	1024

/* Struct for keeping track of content of RR in Answer section */
struct mdns_answer_info {
	uint8_t name[NETPROX_MDNS_MAX_NAME_SZ];	/* Domain name */
	uint16_t name_len;			/* Length of domain name
						 * including zero-length NULL
						 */
	uint16_t type;				/* RR type */
	uint16_t class;				/* RR class */
	uint16_t rdlen;				/* Record data length */
	uint8_t rdata[NETPROX_MDNS_MAX_RDATA_SZ];	/* Record data */
	uint32_t ttl;				/* Time to live in second */
};

/* Structure for mDNS Database
 *
 * Please make sure the value of NETPROX_MDNS_MAX_RR_SZ is always set to
 * make sure struct mdns_db is page aligned.
 *
 * The mDNS RR are stored in uint8_t data[NETPROX_MDNS_MAX_RR_SZ] according to
 * below three regions. The first region contains struct mdns_db_hdr. The second
 * region contains information of various types of RR, which represented by many
 * instances of struct mdns_dnssd_ptr_info, struct mdns_ptr_info,
 * struct mdns_txt_info, struct mdns_srv_info, struct mdns_aaaa_info, and
 * struct mdns_a_info. On the other hand, the third region contains many
 * instances of uint8_t mdns_rr.
 *
 * mDNS Database Diagram :-
 * ========================
 *  +-----------------------------------------------+
 *  |      Region 1: Database Header                |
 *  |        struct mdns_db_hdr                     |
 *  +-----------------------------------------------+
 *  |      Region 2: RR Information                 |
 *  |        struct mdns_dnssd_ptr_info             |
 *  |        struct mdns_ptr_info                   |
 *  /        struct mdns_txt_info                   /
 *  |        struct mdns_srv_info                   |
 *  |        struct mdns_aaaa_info                  |
 *  |        struct mdns_a_info                     |
 *  +-----------------------------------------------+
 *  |      Region 3: name, metadata, and rdata      |
 *  /        uint8_t mdns_rr                        /
 *  |                                               |
 *  +-----------------------------------------------+
 */
#define NETPROX_MDNS_MAX_RR_SZ			4092

/* mDNS database */
struct mdns_db {
	uint8_t data[NETPROX_MDNS_MAX_RR_SZ];
	unsigned int sz;
};

/* Type of mDNS RR */
enum mdns_rr_type {
	MDNS_RR_T_DNSSD_PTR,	/* DNS service discovery pointer RR */
	MDNS_RR_T_PTR,		/* Pointer RR */
	MDNS_RR_T_TXT,		/* Text RR */
	MDNS_RR_T_SRV,		/* Service RR */
	MDNS_RR_T_AAAA,		/* IPv6 RR */
	MDNS_RR_T_A,		/* IPv4 RR */
	MDNS_RR_T_MAX,		/* Total number of RR type */
};

/* mDNS database header */
struct mdns_db_hdr {
	uint8_t mac4_maddr[NP_MAC_ADDR_BYTES];	/* Multicast MAC addr (IPv4) */
	uint8_t ipv4_maddr[NP_IPV4_ADDR_BYTES];	/* Multicast IPv4 addr */
	uint8_t mac6_maddr[NP_MAC_ADDR_BYTES];	/* Multicast MAC addr (IPv6) */
	uint8_t ipv6_maddr[NP_IPV6_ADDR_BYTES];	/* Multicast IPv6 addr */
	uint16_t rr_info[MDNS_RR_T_MAX];	/* Pos of the 1st RR information
						 * structure
						 */
	uint16_t rr_info_count[MDNS_RR_T_MAX];	/* Total number of entries of
						 * RR information structure
						 */
};

/* Type of mDNS PTR RR
 *
 * Special Note:
 * For PTR query, there maybe one or more different query types.
 * For example, in IPP PTR service, the queries are :-
 *  a) _ipp._tcp.local (MDNS_PTR_T_GEN)
 *  b) _universal._sub._ipp._tcp.local (MDNS_PTR_T_UNI)
 *  c) _cups._sub._ipp._tcp.local (MDNS_PTR_T_CUP)
 *  d) _print._sub._ipp._tcp.local (MDNS_PTR_T_PRINT)
 */
enum mdns_ptr_type {
	MDNS_PTR_T_GEN,
	MDNS_PTR_T_UNI,
	MDNS_PTR_T_CUP,
	MDNS_PTR_T_PRINT,
	MDNS_PTR_T_MAX,
};

/* Type of mDNS content */
enum mdns_ct_type {
	MDNS_CT_T_FULL,			/* Normal content (FQDN) */
	MDNS_CT_T_CMPRS,		/* Partial compressed content only */
	MDNS_CT_T_MAX,			/* Total number of content type */
};

/* mDNS name in mdns_rr region
 *
 * Special Note:
 * RR name is stored in the position pointed by 'val'.
 *
 * For mDNS query, the name is always compared in 'full' content type. For mDNS
 * response, the name of the first Answer RR is always 'full' content type and
 * all subsequent Answer RRs are either 'fully compressed' or 'partial
 * compressed' content type. Since 'fully compressed' content type always has
 * 'len' = 0, therefore, the value in 'len' array contains the lengths of 'full'
 * and 'partial compressed' content types.
 *
 * Note: refer to RFC1035 Section 4.1.4 for understanding of message compression
 *       technique.
 */
struct mdns_name {
	uint16_t val;
	uint16_t len[MDNS_CT_T_MAX];
};

/* mDNS DNS-SD PTR RR information
 *
 * Special Note:
 * For _services._dns-sd._udp.local query, there maybe one or more Answer RRs.
 * So, 'answer' points to the position of the first Answer RR, 'answer_len'
 * is the total length of the Answer RRs whereby message compression has been
 * applied, and 'ancount' is the total number of Answer RRs.
 */
struct mdns_dnssd_ptr_info {
	struct mdns_name name;
	uint16_t answer;
	uint16_t answer_len;
	uint16_t ancount;
};

/* mDNS PTR RR information in mdns_rr region
 *
 * Special Note:
 * For mDNS PTR query, there maybe one or more different query types (refer to
 * enum mdns_ptr_type). These queries have different qnames and are stored in
 * different position. Therefore, 'name' is designed to accommodate qname of all
 * PTR query types.
 *
 * The 'metadata' field is common across all queries and responses.
 *
 * All of these mDNS PTR queries have the same announced service PTR name, as
 * stored in 'rdata' and is always in 'compressed' content type. For mDNS PTR
 * query with known-answer matching, we need the service PTR name to be compared
 * in 'full' content type. Therefore, 'rdlen' is designed for both content
 * types.
 *
 * When including PTR RR in a response packet, we should include TXT and SRV
 * RRs. Therefore, 'txt' is the index of struct mdns_txt_info entry named in
 * the PTR rdata whereas 'srv' is the index of struct mdns_srv_info named in
 * the PTR rdata.
 */
struct mdns_ptr_info {
	struct mdns_name name[MDNS_PTR_T_MAX];
	uint16_t metadata;
	uint16_t rdata;
	uint16_t rdlen[MDNS_CT_T_MAX];
	uint16_t txt;
	uint16_t srv;
};

/* mDNS TXT RR information in mdns_rr region
 *
 * Special Note:
 * For mDNS TXT query, the query name belongs to specific service name and has
 * a specific service resource (colour, duplex mode, etc) described in text.
 *
 * For IPP example, query name = <printer name>@<host name>._ipp._tcp.local
 */
struct mdns_txt_info {
	struct mdns_name name;		/* RR name */
	uint16_t metadata;		/* Pos of metadata */
	uint16_t rdata;			/* Pos of rdata */
	uint16_t rdlen;			/* Length of rdata */
};

/* mDNS SRV RR information in mdns_rr region
 *
 * Special Note:
 * For mDNS SRV query, the query name belongs to specific service name and has
 * a specific service resource (priority, weight, port & target) described
 * in SRV format.
 *
 * For IPP example, query name = <printer name>@<host name>._ipp._tcp.local
 *
 * All mDNS SRV response has 'rdata' in 'compressed' content type only.
 * For mDNS SRV query with known-answer matching, we need the SRV rdata to be
 * compared in 'full' content type. Therefore, 'rdlen' is designed for both
 * content types.
 *
 * When including SRV RR in a response packet, we should include address record.
 * Therefore, 'aaaa' is the index of struct mdns_aaaa_info entry named in
 * the SRV rdata whereas 'a' is the index of struct mdns_a_info named in
 * the SRV rdata.
 */
struct mdns_srv_info {
	struct mdns_name name;
	uint16_t metadata;
	uint16_t rdata;
	uint16_t rdlen[MDNS_CT_T_MAX];
	uint16_t aaaa;
	uint16_t a;
};

/* mDNS AAAA (IPv6) RR information in mdns_rr region
 *
 * Special Note:
 * For mDNS SRV response, mDNS AAAA RR is always located after mDNS SRV RR.
 * Therefore, the RR name is always in 'compressed' content type and contains
 * offset to the target (inside rdata) of mDNS SRV RR.
 */
struct mdns_aaaa_info {
	uint16_t metadata;		/* Pos of metadata */
	uint16_t rdata;			/* Pos of rdata (IPv6 address) */
};

/* mDNS A (IPv4) information in mdns_rr region
 *
 * Special Note:
 * For mDNS SRV response, mDNS A RR is always located after mDNS SRV RR.
 * Therefore, the RR name is always in 'compressed' content type and contains
 * offset to the target (inside rdata) of mDNS SRV RR.
 */
struct mdns_a_info {
	uint16_t metadata;		/* Pos of metadata */
	uint16_t rdata;			/* Pos of rdata (IPv4 address) */
};

/* Struct mdns_resp_rr_info is used to record the information needed to
 * retrieve RR information structure which requires response from mDNS database.
 *
 *  a) rr_type: the type of mDNS query (refer to enum mdns_rr_type).
 *
 *  b) rr_entry_index: the index of RR information structure entries.
 *     Based on the mDNS query type (rr_type), mDNS classifier will identify
 *     which index of RR information structure (from mDNS query) requires mDNS
 *     response. It will store the index of that RR information structure in
 *     'rr_entry_index'.
 *
 *  c) ptr_t: the type of PTR query (only applicable for PTR RR).
 *     For mDNS PTR query, different query types (refer to enum mdns_ptr_type)
 *     have different RR names. Therefore, 'ptr_t' is used to record which RR
 *     name is required.
 */
struct mdns_resp_rr_info {
	uint16_t rr_type;
	uint16_t rr_entry_index;
	uint16_t ptr_t;
};

/* Struct mdns_resp_construct_info is used to record the information needed to
 * construct mDNS response message at frame responder.
 *
 *  a) hdr: offset of mDNS header in the response frame.
 *
 *  b) tld: offset of top level domain of 1st Answer RR relative to 'hdr'.
 *          Note: TLD is always '.local'.
 *
 *  c) sld: offset of second level domain of 1st Answer RR relative to 'hdr'.
 *          Note: SLD is usually '._tcp' for printing services.
 *
 *  d) ancount: total number of RRs in Answer Section.
 *
 *  e) mdns_pos: Position of mDNS response message (used to keep tracking the
 *               current position of response packet).
 *
 *  f) pkt_len: Total length of mDNS response message (including L2 - L4).
 *
 *  g) is_ipv6: 1: IPv6 mDNS query & 0: Ipv4 mDNS query.
 */
struct mdns_resp_construct_info {
	uint16_t hdr;
	uint16_t tld;
	uint16_t sld;
	uint16_t ancount;
	uint8_t *mdns_pos;
	int pkt_len;
	bool is_ipv6;
};

/* mDNS Response Message information
 *
 * Special Note:
 * Struct mdns_resp_info is used to record the information needed to
 * construct mDNS response message at frame responder.
 *
 *  a) rr: info needed to retrieve RR information structure from database.
 *
 *  b) construct: information needed to construct mDNS response message.
 *
 *  c) need_ans: the total number of mDNS queries that require answer.
 *
 *     NETPROX_MDNS_QUERY_TO_BE_HANDLE_MAX is the maximum number of mDNS queries
 *     which the Network Proxy classifier can handle.
 *     NETPROX_MDNS_QUERY_TO_BE_HANDLE_MAX is defined as 10 because currently we
 *     only support three types of mDNS query (PTR, SRV, and TXT) on three types
 *     of printing sevices, i.e., UNIX Printer, Internet Printer (IPP), and
 *     Secure Internet Printer (IPPS).
 */
#define NETPROX_MDNS_QUERY_TO_BE_HANDLE_MAX                    10

struct mdns_resp_info {
	struct mdns_resp_rr_info rr[NETPROX_MDNS_QUERY_TO_BE_HANDLE_MAX];
	struct mdns_resp_construct_info construct;
	uint16_t need_ans;
};

/* mDNS domain name is represented as a sequence of labels. Each label is
 * prefixed by the length of that label, as shown in diagram below.
 *
 * mDNS Label Format :-
 * ====================
 *  00 01 02 03 04 05 06 07        LEN-bytes
 * +--+--+--+--+--+--+--+--+-----------/-----------+
 * |0 |0 |       LEN       |         Label         |
 * +--+--+--+--+--+--+--+--+-----------/-----------+
 *
 * When the two most significant bits (MSBs) are set, the following 14 bits
 * represent the offset of compression pointer, as shown in diagram below.
 * For details, refer to RFC1035 Section 4.1.4.
 *
 * mDNS Compression Pointer Format :-
 * ==================================
 *  00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 * |1 |1 |                OFFSET                   |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 */
#define NETPROX_MDNS_QNAME_CT_MASK			0xC0
#define NETPROX_MDNS_QNAME_CT_CMPRS			0xC0
#define NETPROX_MDNS_QNAME_CT_LABEL			0x00
#define NETPROX_MDNS_QNAME_CT_LABEL_LEN_SZ		1
#define NETPROX_MDNS_QNAME_CT_CMPRS_OFFSET_SZ		2

/* Get the upper 8 bits of compression offset */
#define MDNS_GET_UPPER_BYTE_CMPRS_OFFSET(p_name)			       \
	(((p_name) & ~NETPROX_MDNS_QNAME_CT_MASK) << 8)

/* Set the compression offset by set the 2 MSBs */
#define MDNS_SET_CMPRS_OFFSET(offset)	(0xC0 | htons(offset))

/* Cast to respective RR information struct based on the offset of that RR
 * information structure with respect to the first structure of same RR type.
 */
#define MDNS_DNSSD_INFO_PTR(offset)					       \
	(((struct mdns_dnssd_ptr_info *)				       \
	&mdns_db.data[mdns_db_hdr->rr_info[MDNS_RR_T_DNSSD_PTR]]) + (offset))
#define MDNS_PTR_INFO_PTR(offset)					       \
	(((struct mdns_ptr_info *)					       \
	&mdns_db.data[mdns_db_hdr->rr_info[MDNS_RR_T_PTR]]) + (offset))
#define MDNS_TXT_INFO_PTR(offset)					       \
	(((struct mdns_txt_info *)					       \
	&mdns_db.data[mdns_db_hdr->rr_info[MDNS_RR_T_TXT]]) + (offset))
#define MDNS_SRV_INFO_PTR(offset)					       \
	(((struct mdns_srv_info *)					       \
	&mdns_db.data[mdns_db_hdr->rr_info[MDNS_RR_T_SRV]]) + (offset))
#define MDNS_AAAA_INFO_PTR(offset)					       \
	(((struct mdns_aaaa_info *)					       \
	&mdns_db.data[mdns_db_hdr->rr_info[MDNS_RR_T_AAAA]]) + (offset))
#define MDNS_A_INFO_PTR(offset)						       \
	(((struct mdns_a_info *)					       \
	&mdns_db.data[mdns_db_hdr->rr_info[MDNS_RR_T_A]]) + (offset))

/* Get the total number of entries of RR information structure based on
 * enum mdns_rr_type
 */
#define MDNS_GET_RR_COUNT(mdns_rr_type)					       \
	mdns_db_hdr->rr_info_count[(mdns_rr_type)]

/* mDNS SRV type Record Data Format :-
 * ===================================
 *  00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+  --
 * |                   Priority                    |   |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+   |
 * |                    Weight                     |   } NETPROX_MDNS_SRV_PWP_SZ
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+   |	(6 bytes)
 * |                     Port                      |   |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+  --
 * |                                               |
 * /                    Target                     /
 * |                                               |
 * +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
 */
#define NETPROX_MDNS_SRV_PWP_SZ				6

/* Get offset of target based on offset of SRV rdata */
#define MDNS_GET_SRV_TARGET_OFFSET(rdata_offset)			       \
	((rdata_offset) + NETPROX_MDNS_SRV_PWP_SZ)

/* The name of first Answer RR must be a fully qualified domain name, which its
 * last 12 bytes are shown in diagram below:
 *
 *       12  11  10   9   8   7   6   5   4   3   2   1  bytes
 * ~/---+---+---+---+---+---+---+---+---+---+---+---+---+---------/~
 *      |LEN| _   t   c   p |LEN| l   o   c   a   l |00 | metadata
 * ~/---+---+---+---+---+---+---+---+---+---+---+---+---+---------/~
 *      |<-------SLD------->|<---------TLD--------->|
 *
 * Therefore, we can get offset of top level domain (TLD) and second level
 * domain (SLD) based on offset of metadata.
 *
 * Note:
 * The answer RR for dns-sd query cannot be used because its last 12 bytes are
 * ._udp.local.
 */
#define MDNS_GET_TLD_OFFSET(metadata_offset)				       \
	((metadata_offset) - 7)
#define MDNS_GET_SLD_OFFSET(metadata_offset)				       \
	((metadata_offset) - 12)

#define MDNS_KNOWN_ANS_CONFLIT_FOUND			-1
#define MDNS_KNOWN_ANS_DATA_MATCHED			1

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_ETHERNET_ETH_DWC_EQOS_NETPROX_PROTO_H_ */
