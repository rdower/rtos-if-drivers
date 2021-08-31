/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(eth_vnic, CONFIG_ETHERNET_LOG_LEVEL);

#include <kernel.h>
#include <device.h>
#include <sys/__assert.h>
#include <sys/util.h>
#include <errno.h>
#include <stdbool.h>
#include <net/net_pkt.h>
#include <net/net_if.h>
#include <net/ethernet.h>
#include <ethernet/eth_stats.h>
#include <device.h>
#include <init.h>
#include <string.h>
#include "heci.h"

#define VNIC_MTU                       940
#define EHECI_MSG_DATA                 0x1
#define EHECI_MSG_EVENT                0x2
#define EHECI_ISH_READ                 0x1
#define EHECI_ISH_WRITE                0x2
#define EHECI_ISH_HEADER_VERSION 0
#define EHECI_DATA_BUFLEN     VNIC_MTU
#define EHECI_RX_MAX_LEN      VNIC_MTU
#define EHECI_TX_MAX_LEN      VNIC_MTU
#define VNIC_NUM_RX_MSG_Q_ENTRIES (8)

struct mq_item {
	uint8_t buffer[VNIC_MTU];
	uint32_t len;
	bool in_use;
};

struct mq_item mq_entry[VNIC_NUM_RX_MSG_Q_ENTRIES];

K_MUTEX_DEFINE(vnic_tx_mutex);

#define HECI_CLIENT_VNIC_GUID { 0xeb83e1fb, 0x4c61, 0x4829, \
				{ 0x98, 0x4c, 0x3, 0x23,    \
				  0xab, 0x4b, 0x41, 0x65 } }


#define VNIC_MAX_TX_SIZE        EHECI_TX_MAX_LEN
#define VNIC_MAX_RX_SIZE        EHECI_RX_MAX_LEN
#define VNIC_TASK_STACK_SIZE         (2000)
#define VNIC_TASK_PRIORITY      (7)
#define VNIC_MAX_CON            1
#define VNIC_TX_TIMEOUT_MS      K_MSEC(2000)

#define DEV_DATA(dev) ((struct vnic_dev_data *)((dev)->data))

static const struct device *vnic_dev_ptr;

struct eheci_msg_hdr {
	uint32_t version : 2;
	uint32_t data_type : 2;
	uint32_t request_type : 2;
	uint32_t offset;
	uint32_t data_len;
	uint32_t event;
};

struct eheci_msg {
	struct eheci_msg_hdr hdr;
	uint8_t payload[EHECI_DATA_BUFLEN];
};

static heci_rx_msg_t vnic_rx_msg;
static uint8_t vnic_rx_buffer[VNIC_MAX_RX_SIZE];
static uint8_t vnic_tx_buffer[VNIC_MAX_TX_SIZE];

K_MSGQ_DEFINE(vnic_msgq, sizeof(struct mq_item *), VNIC_NUM_RX_MSG_Q_ENTRIES,
				4);

static K_THREAD_STACK_DEFINE(vnic_stack, VNIC_TASK_STACK_SIZE);
static struct k_thread vnic_thread;
static uint32_t vnic_event;
static uint32_t vnic_conn_id;

struct vnic_dev_data {
	struct net_if *iface;
	uint8_t mac_addr[6];
};

struct vnic_dev_cfg {
	uint32_t *heci_ptr;
	uint32_t *other_data;
};

static struct vnic_dev_data vnic0_data = {
	.mac_addr = {
		CONFIG_ETH_PSE_VNIC_MAC0,
		CONFIG_ETH_PSE_VNIC_MAC1,
		CONFIG_ETH_PSE_VNIC_MAC2,
		CONFIG_ETH_PSE_VNIC_MAC3,
		CONFIG_ETH_PSE_VNIC_MAC4,
		CONFIG_ETH_PSE_VNIC_MAC5
	}
};

static const struct vnic_dev_cfg vnic0_config = {
	.heci_ptr = NULL,
	.other_data = NULL
};

static int vnic_connect_done;

static int vnic_process_rx(uint8_t *buffer, uint16_t frm_len);

static volatile int buf_count;

static void vnic_event_callback(uint32_t event, void *param)
{
	uint8_t *buf;
	int idx;
	struct mq_item *qi_ptr;

	switch (event) {
	case HECI_EVENT_NEW_MSG:
		if (vnic_rx_msg.msg_lock != MSG_LOCKED) {
			LOG_ERR("invalid heci message");
			break;
		}

		if (vnic_rx_msg.type == HECI_CONNECT) {
			vnic_conn_id = vnic_rx_msg.connection_id;
			vnic_connect_done = true;
			LOG_DBG("New Connection ID: %u", vnic_conn_id);

		} else if (vnic_rx_msg.type == HECI_REQUEST) {
			idx = (buf_count++) % VNIC_NUM_RX_MSG_Q_ENTRIES;
			if (mq_entry[idx].in_use != true) {
				mq_entry[idx].in_use = true;
				buf = mq_entry[idx].buffer;
				mq_entry[idx].len = vnic_rx_msg.length;
				memcpy((void *)buf, (void *)vnic_rx_msg.buffer,
						vnic_rx_msg.length);
				qi_ptr = &mq_entry[idx];
				if (k_msgq_put(&vnic_msgq, &qi_ptr, K_NO_WAIT)
						!= 0) {
					LOG_ERR("Adding to vnic_msgq failed");
				}
			} else {
				LOG_ERR("No mq buffer available");
			}
		}

		/*
		 * Send flow control after finishing one message,
		 * allow host to send new request
		 */
		heci_send_flow_control(vnic_conn_id);
		break;

	case HECI_EVENT_DISCONN:
		LOG_DBG("Disconnect request conn %d",
				vnic_conn_id);
		heci_complete_disconnect(vnic_conn_id);
		break;

	default:
		LOG_ERR("Wrong heci event %u", vnic_event);
		break;
	}

}

static int vnic_process_rx(uint8_t *buffer, uint16_t frm_len)
{
	struct vnic_dev_data *const dev_data = DEV_DATA(vnic_dev_ptr);
	struct net_pkt *pkt;
	int r;

	struct net_buf *frag;

	if (frm_len == 0) {
		LOG_ERR("Frame too small: %u", frm_len);
		goto error;
	}

	pkt = net_pkt_rx_alloc_with_buffer(dev_data->iface, frm_len,
					   AF_UNSPEC, 0, K_NO_WAIT);
	if (!pkt) {
		LOG_ERR("Failed to obtain rx pkt");
		goto error;
	}

	if (net_pkt_write(pkt, INT_TO_POINTER((uint32_t) buffer), frm_len)) {
		LOG_ERR("Out of memory for rx buffer");
		net_pkt_unref(pkt);
		pkt = NULL;
		goto error;
	}

	frag = pkt->frags;

	LOG_HEXDUMP_DBG(frag->data, frag->len, "RX_DATA");
	r = net_recv_data(dev_data->iface, pkt);
	if (r < 0) {
		LOG_ERR("Failed to enqueue frame into RX queue: %d", r);
		net_pkt_unref(pkt);
		goto error;
	}

	LOG_DBG("Added packet to receive data");
	return 0;

error:
	eth_stats_update_errors_rx(dev_data->iface);
	LOG_ERR("Recv error");
	return -1;
}

static void vnic_task(void *p1, void *p2, void *p3)
{
	struct mq_item *mq_rx = NULL;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_msgq_get(&vnic_msgq, &mq_rx, K_FOREVER);
		LOG_DBG("RXLEN =%x", mq_rx->len);
		vnic_process_rx(mq_rx->buffer, mq_rx->len);
		mq_rx->in_use = 0;
	}

}

static int vnic_init(int arg)
{
	int ret;
	heci_client_t vnic_client = {
		.protocol_id = HECI_CLIENT_VNIC_GUID,
		.max_msg_size = VNIC_MAX_TX_SIZE,
		.protocol_ver = 1,
		.max_n_of_connections = VNIC_MAX_CON,
		.dma_header_length = 0,
		.dma_enabled = false,
		.rx_buffer_len = VNIC_MAX_RX_SIZE,
		.event_cb = vnic_event_callback,
	};

	ARG_UNUSED(arg);

	vnic_client.rx_msg = &vnic_rx_msg;
	vnic_client.rx_msg->buffer = vnic_rx_buffer;

	ret = heci_register(&vnic_client);
	if (ret) {
		LOG_ERR("Failed to register vnic client %d", ret);
		return ret;
	}

	LOG_DBG("Heci Regn successful");
	k_thread_create(&vnic_thread, vnic_stack,
			K_THREAD_STACK_SIZEOF(vnic_stack), vnic_task,
			NULL, NULL, NULL, VNIC_TASK_PRIORITY, 0, K_MSEC(0));

	return 0;
}

static int vnic_tx(const struct device *dev, struct net_pkt *pkt)
{
	struct net_buf *frag;
	mrd_t m = { 0 };
	int total_len = 0;
	int ret;

	__ASSERT(pkt, "buf pointer is NULL");
	__ASSERT(pkt->frags, "Frame data missing");

	LOG_DBG("Sending TX net packet");

	if (vnic_connect_done == 0) {
		LOG_ERR("Connection not ready, tx failure");
		return -1;
	}

	ret = k_mutex_lock(&vnic_tx_mutex, VNIC_TX_TIMEOUT_MS);
	if (ret != 0) {
		LOG_ERR("VNIC mutex timeout");
		return -1;
	}

	frag = pkt->frags;

	while (frag) {
		memcpy((void *)(vnic_tx_buffer + total_len),
				(void *)frag->data, frag->len);
		total_len = total_len + frag->len;
		frag = frag->frags;
	}

	m.buf = vnic_tx_buffer;
	m.len = total_len;

	LOG_DBG("vnic heci tx: Total pkt size =%x", m.len);
	LOG_HEXDUMP_DBG(vnic_tx_buffer, m.len, "TX_DATA");

	if (!heci_send(vnic_conn_id, &m)) {
		LOG_ERR("Tx Failure");
		ret = -1;
		goto end;
	}

	LOG_DBG("Tx_Success");
	k_yield();
	ret = 0;
end:
	k_mutex_unlock(&vnic_tx_mutex);
	return ret;
}

static int vnic_initialize(const struct device *dev)
{
	vnic_init(0);
	return 0;
}

static void vnic0_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	static bool init_done;
	struct vnic_dev_data *dev_data;

	LOG_DBG("VNIC IFACE INIT");
	if (dev == NULL) {
		LOG_ERR("dev NULL");
		return;
	}

	dev_data = DEV_DATA(dev);
	vnic_dev_ptr = dev;

	if (dev_data == NULL) {
		LOG_ERR("Dev data is null");
		return;
	}

	LOG_DBG("VNIC IFACE = %x", (unsigned int)&dev_data->iface);
	dev_data->iface = (struct net_if *)iface;

	/* The rest of initialization should only be done once */
	if (init_done) {
		LOG_ERR("Reinit attempted");
		return;
	}

	ethernet_init(iface);
	LOG_INF("VNIC MAC: %02x:%02x:%02x:%02x:%02x:%02x",
		dev_data->mac_addr[0], dev_data->mac_addr[1],
		dev_data->mac_addr[2], dev_data->mac_addr[3],
		dev_data->mac_addr[4], dev_data->mac_addr[5]);

	/* Register Ethernet MAC Address with the upper layer */
	net_if_set_link_addr(iface, dev_data->mac_addr,
			     sizeof(dev_data->mac_addr),
			     NET_LINK_ETHERNET);
	init_done = true;
}

/** Get the device capabilities */
enum ethernet_hw_caps vnic_get_capabilities(const struct device *dev)
{
	return ETHERNET_DUPLEX_SET | ETHERNET_HW_RX_CHKSUM_OFFLOAD;
}

static const struct ethernet_api vnic_api = {
	.iface_api.init = vnic0_iface_init,
	.get_capabilities = vnic_get_capabilities,
	.send = vnic_tx
};

ETH_NET_DEVICE_INIT(vnic0_pse, "VNIC_0", vnic_initialize, NULL,
		    &vnic0_data, &vnic0_config, CONFIG_ETH_INIT_PRIORITY,
		    &vnic_api, VNIC_MTU);

