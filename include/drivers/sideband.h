/*
 * Copyright (c) 2021 Intel Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SIDEBAND_H_
#define ZEPHYR_INCLUDE_SIDEBAND_H_

/**
 * @file
 * @brief Public SIDEBAND driver APIs
 */

#include <zephyr/types.h>
#include <device.h>


/* @brief Define sideband port number macro */
#define SIDEBAND_PORT_CSME (0)
#define SIDEBAND_PORT_PMC (1)
#define SIDEBAND_PORT_PSE_BRIDGE (4)
#define SIDEBAND_PORT_DRNG (18)

/* @brief Define action for message */
#define SIDEBAND_ACTION_WRITE (0)
#define SIDEBAND_ACTION_READ (1)

/* @brief Define client number for register client api */
#define SIDEBAND_DOWN_PMC (0)
#define SIDEBAND_DOWN_TSYNC (1)
#define SIDEBAND_DOWN_CSME (4)

/**
 * @brief Sideband raw message structure
 *
 * This structure used to send raw message.
 *
 * @param addr_low Access address low 32 bits.
 * @param addr_high Access address high 32 bits.
 * @param attr Attribute value of message received.
 * @param data0 Low 32-bit data received.
 * @param data1 High 32-bit data, this value shall be cmd value while send.
 * @param sairs Sai value from the source port.
 */
struct sb_raw_message {
	uint32_t addr_low;
	uint32_t addr_high;
	uint32_t attr;
	uint32_t data0;
	uint32_t data1;
	uint32_t sairs;
};

/**
 * @typedef sideband_api_send()
 * @brief Callback API for sideband send message.
 *
 * @see sideband_send() for argument descriptions.
 */
typedef int (*sideband_api_send)(const struct device *dev, uint32_t port,
				 uint32_t action, uint64_t addr, uint32_t data);

/**
 * @typedef sideband_api_receive()
 * @brief Callback API for receive sideband message.
 *
 * @see sideband_receive() for argument descriptions.
 */
typedef int (*sideband_api_receive)(const struct device *dev, uint32_t port,
				    uint64_t *addr, uint64_t *data);

/**
 * @typedef sideband_api_send_ack()
 * @brief Callback API for sideband send ACK.
 *
 * @see sideband_send_ack() for argument descriptions.
 */
typedef int (*sideband_api_send_ack)(const struct device *dev, uint32_t port,
				     uint32_t action, uint32_t data);

/**
 * @typedef sideband_api_wait_ack()
 * @brief Callback API for wait an ACK from other port.
 *
 * @see sideband_wait_ack() for argument descriptions.
 */
typedef int (*sideband_api_wait_ack)(const struct device *dev, uint32_t port,
				     uint32_t action, uint32_t *data);

/**
 * @typedef sideband_api_write_raw()
 * @brief Callback API for sideband send raw message.
 *
 * @see sideband_write_raw() for argument descriptions.
 */
typedef int (*sideband_api_write_raw)(const struct device *dev,
				      struct sb_raw_message *message);

/**
 * @typedef sideband_api_read_raw()
 * @brief Callback API for sideband read downstream raw message.
 *
 * @see sideband_read_raw() for argument descriptions.
 */
typedef int (*sideband_api_read_raw)(const struct device *dev,
				     struct sb_raw_message *message);

/**
 * @typedef sideband_api_register_client()
 * @brief Callback API for sideband to register an downstream client.
 *
 * @see sideband_register_client() for argument descriptions.
 */
typedef int (*sideband_api_register_client)(const struct device *dev,
					    uint32_t client,
					    uint32_t opcode);

/**
 * @typedef sideband_api_unregister_client()
 * @brief Callback API for sideband to unregister a client.
 *
 * @see sideband_unregister_client() for argument descriptions.
 */
typedef int (*sideband_api_unregister_client)(const struct device *dev,
					      uint32_t client);


/**
 * @brief SIDEBAND driver API
 *
 * This is the mandatory API any SIDEBAND driver needs to expose.
 */
__subsystem struct sideband_driver_api {
	sideband_api_send send;
	sideband_api_receive receive;
	sideband_api_send_ack send_ack;
	sideband_api_wait_ack wait_ack;
	sideband_api_write_raw write_raw;
	sideband_api_read_raw read_raw;
	sideband_api_register_client register_client;
	sideband_api_unregister_client unregister_client;
};

/**
 * @brief Send sideband message to other port
 *
 * This routine send sideband message to other port.
 *
 * @param dev SIDEBAND device
 * @param port port number.
 * @param action Read or write action.
 * @param addr 64-bits address, can be 0 if not needed.
 * @param data 32-bit data need to send, can be 0 if action is read.
 * @return 0 on success, negative on error
 */
__syscall int sideband_send(const struct device *dev, uint32_t port,
			    uint32_t action, uint64_t addr, uint32_t data);

static inline int z_impl_sideband_send(const struct device *dev, uint32_t port,
				       uint32_t action, uint64_t addr,
				       uint32_t data)
{
	const struct sideband_driver_api *api = dev->api;

	return api->send(dev, port, action, addr, data);
}

/**
 * @brief Receive sideband message from other port
 *
 * This routine receive sideband message from other port. Before call this api
 * users need to call sideband_register_client api.
 *
 * @param dev SIDEBAND device
 * @param port port number.
 * @param addr Pointer to 64-bits variable to receive address information.
 * @param data Pointer to 64-bit variable to receive data.
 * @return 0 on success, negative on error
 */
__syscall int sideband_receive(const struct device *dev, uint32_t port,
			       uint64_t *addr, uint64_t *data);

static inline int z_impl_sideband_receive(const struct device *dev,
					  uint32_t port, uint64_t *addr,
					  uint64_t *data)
{
	const struct sideband_driver_api *api = dev->api;

	return api->receive(dev, port, addr, data);
}

/**
 * @brief Send sideband ACK message to other port
 *
 * This routine send ACK message to other port.
 *
 * @param dev SIDEBAND device
 * @param port port number.
 * @param action Read or write action.
 * @param data 32-bit data need to send, can be 0 if ACK with no data.
 * @return 0 on success, negative on error
 */
__syscall int sideband_send_ack(const struct device *dev, uint32_t port,
				uint32_t action, uint32_t data);

static inline int z_impl_sideband_send_ack(const struct device *dev,
					   uint32_t port, uint32_t action,
					   uint32_t data)
{
	const struct sideband_driver_api *api = dev->api;

	return api->send_ack(dev, port, action, data);
}

/**
 * @brief Wait an ACK message from other port
 *
 * This routine wait ACK message from other port.
 *
 * @param dev SIDEBAND device
 * @param port port number.
 * @param action Read or write action.
 * @param data Pointer to 32-bit data to receive data.
 * @return 0 on success, negative on error
 */
__syscall int sideband_wait_ack(const struct device *dev, uint32_t port,
				uint32_t action, uint32_t *data);

static inline int z_impl_sideband_wait_ack(const struct device *dev,
					   uint32_t port, uint32_t action,
					   uint32_t *data)
{
	const struct sideband_driver_api *api = dev->api;

	return api->wait_ack(dev, port, action, data);
}

/**
 * @brief Read sideband downstream raw message from other port
 *
 * This routine read raw sideband message from other port, but need user know
 * exactly all these parameters, users can read all information of the message.
 *
 * @param dev SIDEBAND device
 * @param message Sideband raw message structure pointer.
 * @return 0 on success, negative on error
 */
__syscall int sideband_read_raw(const struct device *dev,
				struct sb_raw_message *message);

static inline int z_impl_sideband_read_raw(const struct device *dev,
					   struct sb_raw_message *message)
{
	const struct sideband_driver_api *api = dev->api;

	return api->read_raw(dev, message);
}

/**
 * @brief Send sideband raw message to other port
 *
 * This routine send sideband message to other port, but need user know exactly
 * all these parameters, users can send message with specific opcode.
 *
 * @param dev SIDEBAND device
 * @param message Sideband raw message structure pointer.
 * @return 0 on success, negative on error
 */
__syscall int sideband_write_raw(const struct device *dev,
				 struct sb_raw_message *message);

static inline int z_impl_sideband_write_raw(const struct device *dev,
					    struct sb_raw_message *message)
{
	const struct sideband_driver_api *api = dev->api;

	return api->write_raw(dev, message);
}

/**
 * @brief Register a client for sideband receive action
 *
 * This routine register a client for sideband receive.
 *
 * @param dev SIDEBAND device
 * @param client Client number to be registered.
 * @param opcode Opcode of this receive information
 * @return 0 on success, negative on error
 */
__syscall int sideband_register_client(const struct device *dev,
				       uint32_t client, uint32_t opcode);

static inline int z_impl_sideband_register_client(const struct device *dev,
						  uint32_t client,
						  uint32_t opcode)
{
	const struct sideband_driver_api *api = dev->api;

	return api->register_client(dev, client, opcode);
}

/**
 * @brief Unregister a client
 *
 * This routine unregister a client for sideband receive.
 *
 * @param dev SIDEBAND device
 * @param client Client number to be registered.
 * @return 0 on success, negative on error
 */
__syscall int sideband_unregister_client(const struct device *dev,
					 uint32_t client);

static inline int z_impl_sideband_unregister_client(const struct device *dev,
						    uint32_t client)
{
	const struct sideband_driver_api *api = dev->api;

	return api->unregister_client(dev, client);
}

#include <syscalls/sideband.h>

#endif  /* ZEPHYR_INCLUDE_SIDEBAND_H_ */
