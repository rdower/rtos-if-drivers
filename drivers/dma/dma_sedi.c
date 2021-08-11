/*
 * Copyright (c) 2018-2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT intel_sedi_dma

#include <errno.h>
#include <stdio.h>
#include <kernel.h>
#include <device.h>
#include <string.h>
#include <init.h>
#include <drivers/dma.h>
#include <devicetree.h>
#include <cache.h>

#include "sedi_driver.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(dma_sedi, CONFIG_DMA_LOG_LEVEL);

#define CACHE_MASK 31

#define BLK_SLAB_COUNT 128
#define BLK_SLAB_ALIGN 32
#define GET_MSB(data64) ((uint32_t)(data64 >> 32))
#define GET_LSB(data64) ((uint32_t)(data64))
#define BUILD_64_VAR(up32, low32) ((uint64_t)up32 << 32 | (uint64_t)low32)

#if CONFIG_DMA_MULTIPLE_BLOCK
#define LL_SLAB_COUNT 128
#define LL_SLAB_ALIGN 8
K_MEM_SLAB_DEFINE(ll_mem_slab, sizeof(dma_linked_list_item_t), LL_SLAB_COUNT,
		  LL_SLAB_ALIGN);
static int free_ll_nodes_slab(dma_linked_list_item_t **ll_p_h);
#endif

K_MEM_SLAB_DEFINE(blk_mem_slab, sizeof(struct dma_block_config), BLK_SLAB_COUNT,
		  BLK_SLAB_ALIGN);

extern void dma_isr(sedi_dma_t dma_device);

struct dma_sedi_config_info {
	sedi_dma_t instance; /* Controller instance. */
};

struct dma_sedi_driver_data {
	struct k_sem sema[DMA_CHANNEL_NUM];
	struct dma_config dma_configs[DMA_CHANNEL_NUM];
#if CONFIG_DMA_MULTIPLE_BLOCK
	dma_linked_list_item_t *ll_header[DMA_CHANNEL_NUM];
#endif
	uint8_t power_status;
};

#define DEV_NAME(dev) ((dev)->name)
#define DEV_DATA(dev) ((struct dma_sedi_driver_data *const)(dev)->data)
#define DEV_CFG(dev) \
	((const struct dma_sedi_config_info *const)(dev)->config)

/* map dma num to dma driver pointer*/
static const struct device *dev_map[SEDI_DMA_NUM] = { 0 };

/*
 * this function will be called when dma tranferring is completed
 * or error happened
 */
static void dma_handler(sedi_dma_t dma_device, int channel, int event_id,
			void *args)
{
	ARG_UNUSED(args);
	const struct device *dev = dev_map[dma_device];
	struct dma_sedi_driver_data *const data = DEV_DATA(dev);
	struct dma_config *config = &(data->dma_configs[channel]);

	k_sem_give(&data->sema[channel]);

#if CONFIG_DMA_MULTIPLE_BLOCK
	free_ll_nodes_slab(&(data->ll_header[channel]));
#endif
	sedi_dma_set_power(dma_device, channel, SEDI_POWER_OFF);
	/* run user-defined callback */
	if (config->dma_callback) {
		if ((event_id == SEDI_DMA_EVENT_TRANSFER_DONE) &&
		    (config->complete_callback_en)) {
			config->dma_callback(dev, config->user_data,
					     channel, 0);
		} else if (config->error_callback_en) {
			config->dma_callback(dev, config->user_data,
					     channel, event_id);
		}
	}
}

/* map width to certain macros*/
static int width_index(uint32_t num_bytes, uint32_t *index)
{
	switch (num_bytes) {
	case 1:
		*index = DMA_TRANS_WIDTH_8;
		break;
	case 2:
		*index = DMA_TRANS_WIDTH_16;
		break;
	case 4:
		*index = DMA_TRANS_WIDTH_32;
		break;
	case 8:
		*index = DMA_TRANS_WIDTH_64;
		break;
	case 16:
		*index = DMA_TRANS_WIDTH_128;
		break;
	case 32:
		*index = DMA_TRANS_WIDTH_256;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

/* map burst size to certain macros*/
static int burst_index(uint32_t num_units, uint32_t *index)
{
	switch (num_units) {
	case 1:
		*index = DMA_BURST_TRANS_LENGTH_1;
		break;
	case 4:
		*index = DMA_BURST_TRANS_LENGTH_4;
		break;
	case 8:
		*index = DMA_BURST_TRANS_LENGTH_8;
		break;
	case 16:
		*index = DMA_BURST_TRANS_LENGTH_16;
		break;
	case 32:
		*index = DMA_BURST_TRANS_LENGTH_32;
		break;
	case 64:
		*index = DMA_BURST_TRANS_LENGTH_64;
		break;
	case 128:
		*index = DMA_BURST_TRANS_LENGTH_128;
		break;
	case 256:
		*index = DMA_BURST_TRANS_LENGTH_256;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

#if CONFIG_DMA_MULTIPLE_BLOCK
static dma_linked_list_item_t *alloc_ll_nodes_slab(int count)
{
	int ret = 0;
	dma_linked_list_item_t *ll_p, *ll_p_h = NULL;

	for (int i = 0; i < count; i++) {
		if (k_is_in_isr()) {
			ret = k_mem_slab_alloc(&ll_mem_slab, (void **)&ll_p,
					       K_NO_WAIT);
			if (ret) {
				LOG_ERR("dma resource is used up, try it later!!!");
				free_ll_nodes_slab(&ll_p);
				return NULL;
			}
		} else {
			k_mem_slab_alloc(&ll_mem_slab,
					 (void **)&ll_p, K_FOREVER);
		}
		LOG_DBG("linked list [%d] : %p", i, ll_p);
		memset((void *)ll_p, 0x0, sizeof(dma_linked_list_item_t));
		ll_p->next_ll_p = (const dma_linked_list_item_t *)ll_p_h;
		ll_p_h = ll_p;
	}
	return ll_p_h;
}

static int free_ll_nodes_slab(dma_linked_list_item_t **ll_p_h)
{
	int count = 0;
	dma_linked_list_item_t *p = *ll_p_h, *p_next;

	while (p) {
		p_next = (dma_linked_list_item_t *)p->next_ll_p;
		k_mem_slab_free(&ll_mem_slab, (void **)&p);
		LOG_DBG("linked list [%d] : %p", count, p);
		p = p_next;
		count++;
	}
	*ll_p_h = NULL;
	return count;
}
#endif

static struct dma_block_config *
alloc_and_cpy_blk_nodes_slab(struct dma_block_config *src_blk)
{
	int i = 0;
	struct dma_block_config *blk_p, *blk_p_h = NULL;

	while (src_blk) {
		k_mem_slab_alloc(&blk_mem_slab, (void **)&blk_p, K_FOREVER);
		LOG_DBG("block node [%02d] : %p", i, blk_p);
		memcpy((void *)blk_p, src_blk, sizeof(struct dma_block_config));
		blk_p->next_block = blk_p_h;
		src_blk = src_blk->next_block;
		blk_p_h = blk_p;
		i++;
	}
	return blk_p_h;
}

static int free_blk_nodes_slab(struct dma_config *config)
{
	int count = 0;
	struct dma_block_config *p = config->head_block, *p_next;

	while (p) {
		p_next = (struct dma_block_config *)p->next_block;
		k_mem_slab_free(&blk_mem_slab, (void **)&p);
		LOG_DBG("block node [%d] : %p", count, p);
		p = p_next;
		count++;
	}
	config->head_block = NULL;
	return count;
}

static void dma_config_convert(struct dma_config *config,
			       dma_memory_type_t *src_mem,
			       dma_memory_type_t *dst_mem,
			       uint8_t *sedi_dma_dir)
{

	*src_mem = DMA_SRAM_MEM;
	*dst_mem = DMA_SRAM_MEM;
	*sedi_dma_dir = MEMORY_TO_MEMORY;
	switch (config->channel_direction) {
	case MEMORY_TO_MEMORY:
	case MEMORY_TO_PERIPHERAL:
	case PERIPHERAL_TO_MEMORY:
	case PERIPHERAL_TO_PERIPHERAL:
		*sedi_dma_dir = config->channel_direction;
		break;
	case MEMORY_TO_HOSTDDR:
		*dst_mem = DMA_DRAM_MEM;
		break;
	case HOSTDDR_TO_MEMORY:
		*src_mem = DMA_DRAM_MEM;
		break;
	case MEMORY_TO_IMR:
		*dst_mem = DMA_UMA_MEM;
		break;
	case IMR_TO_MEMORY:
		*src_mem = DMA_UMA_MEM;
		break;
	}
}

/* config basic dma */
static int dma_sedi_apply_common_config(sedi_dma_t dev, uint32_t channel,
					struct dma_config *config, uint8_t *dir)
{
	uint8_t direction = MEMORY_TO_MEMORY;
	dma_memory_type_t src_mem = DMA_SRAM_MEM, dst_mem = DMA_SRAM_MEM;

	dma_config_convert(config, &src_mem, &dst_mem, &direction);

	if (dir) {
		*dir = direction;
	}

	/* configure dma tranferring direction*/
	sedi_dma_control(dev, channel, SEDI_CONFIG_DMA_DIRECTION,
			 direction);

	if (direction == MEMORY_TO_MEMORY) {
		sedi_dma_control(dev, channel, SEDI_CONFIG_DMA_SR_MEM_TYPE,
				 src_mem);
		sedi_dma_control(dev, channel, SEDI_CONFIG_DMA_DT_MEM_TYPE,
				 dst_mem);
	} else if (direction == MEMORY_TO_PERIPHERAL) {
		sedi_dma_control(dev, channel, SEDI_CONFIG_DMA_HS_DEVICE_ID,
				 config->dma_slot);
		sedi_dma_control(dev, channel, SEDI_CONFIG_DMA_HS_POLARITY,
				 DMA_HS_POLARITY_HIGH);
		sedi_dma_control(dev, channel,
				 SEDI_CONFIG_DMA_HS_DEVICE_ID_PER_DIR,
				 DMA_HS_PER_TX);
	} else if (direction == PERIPHERAL_TO_MEMORY) {
		sedi_dma_control(dev, channel, SEDI_CONFIG_DMA_HS_DEVICE_ID,
				 config->dma_slot);
		sedi_dma_control(dev, channel, SEDI_CONFIG_DMA_HS_POLARITY,
				 DMA_HS_POLARITY_HIGH);
		sedi_dma_control(dev, channel,
				 SEDI_CONFIG_DMA_HS_DEVICE_ID_PER_DIR,
				 DMA_HS_PER_RX);
	} else {
		return -1;
	}
	return 0;
}

#if CONFIG_DMA_MULTIPLE_BLOCK
static int dma_sedi_apply_ll_config(sedi_dma_t dev, uint32_t channel,
				    struct dma_config *config,
				    dma_linked_list_item_t **ll_p)
{
	int ret = 0, i, ll_size = 0;
	uint8_t dir;
	uint32_t ctrl_low, src, dst;
	uint32_t src_w, dst_w, src_b, dst_b;
	struct dma_block_config *block = config->head_block;

	ret = dma_sedi_apply_common_config(dev, channel, config, &dir);
	if (ret != 0) {
		goto INVALID_ARGS;
	}

	if (config->channel_direction == MEMORY_TO_HOSTDDR) {
		sedi_dma_control(dev, channel, SEDI_CONFIG_DMA_LL_SR_MSB,
				 GET_MSB(block->source_address));
	}
	if (config->channel_direction == HOSTDDR_TO_MEMORY) {
		sedi_dma_control(dev, channel, SEDI_CONFIG_DMA_LL_DT_MSB,
				 GET_MSB(block->dest_address));
	}
	ret = width_index(config->source_data_size, &src_w);
	if (ret != 0) {
		goto INVALID_ARGS;
	}
	ret = width_index(config->dest_data_size, &dst_w);
	if (ret != 0) {
		goto INVALID_ARGS;
	}
	ret = burst_index(config->source_burst_length, &src_b);
	if (ret != 0) {
		goto INVALID_ARGS;
	}
	ret = burst_index(config->dest_burst_length, &dst_b);
	if (ret != 0) {
		goto INVALID_ARGS;
	}

	dma_linked_list_item_t *ll_p_h =
		alloc_ll_nodes_slab(config->block_count);
	if (ll_p_h == NULL) {
		return -1;
	}
	*ll_p = ll_p_h;

	if ((block->source_gather_en == 1) || (block->dest_scatter_en == 1)) {
		/*scatter gather mode*/
		src = GET_LSB(block->source_address);
		dst = GET_LSB(block->dest_address);
		for (i = 0; i < config->block_count; i++) {
			ll_size += sizeof(dma_linked_list_item_t);
			ctrl_low = BUILD_LL_CTRL_REG(dir,
						     src_b, dst_b, src_w, src_w,
						     block->source_addr_adj,
						     block->dest_addr_adj);
			dma_fill_linkedlist(ll_p_h, src, dst, block->block_size,
					    ctrl_low, ll_p_h->next_ll_p);
			LOG_DBG("filling linked list [%d] : %p", i, ll_p_h);
			ll_p_h = (dma_linked_list_item_t *)ll_p_h->next_ll_p;
			if (block->source_gather_en) {
				switch (block->source_addr_adj) {
				case DMA_ADDR_ADJ_INCREMENT:
					src += block->source_gather_interval;
					break;
				case DMA_ADDR_ADJ_NO_CHANGE:
					break;
				case DMA_ADDR_ADJ_DECREMENT:
					src -= block->source_gather_interval;
					break;
				default:
					ret = -1;
					goto INVALID_ARGS;
				}
				if (block->dest_reload_en) {
					continue;
				}
				switch (block->dest_addr_adj) {
				case DMA_ADDR_ADJ_INCREMENT:
					dst += block->block_size;
					break;
				case DMA_ADDR_ADJ_NO_CHANGE:
					break;
				case DMA_ADDR_ADJ_DECREMENT:
					dst -= block->block_size;
					break;
				default:
					ret = -1;
					goto INVALID_ARGS;
				}
			}
			if (block->dest_scatter_en) {
				switch (block->dest_addr_adj) {
				case DMA_ADDR_ADJ_INCREMENT:
					dst += block->dest_scatter_interval;
					break;
				case DMA_ADDR_ADJ_NO_CHANGE:
					break;
				case DMA_ADDR_ADJ_DECREMENT:
					dst -= block->dest_scatter_interval;
					break;
				default:
					ret = -1;
					goto INVALID_ARGS;
				}
				if (block->source_reload_en) {
					continue;
				}
				switch (block->source_addr_adj) {
				case DMA_ADDR_ADJ_INCREMENT:
					src += block->block_size;
					break;
				case DMA_ADDR_ADJ_NO_CHANGE:
					break;
				case DMA_ADDR_ADJ_DECREMENT:
					src -= block->block_size;
					break;
				default:
					ret = -1;
					goto INVALID_ARGS;
				}
			}
		}

#if CONFIG_CACHE_FLUSHING
		sys_cache_flush((void *)*ll_p,
				ll_size + ((uint32_t)(*ll_p) & CACHE_MASK));
#else
		sedi_core_clean_dcache_by_addr((uint32_t *)*ll_p,
				ll_size + ((uint32_t)(*ll_p) & CACHE_MASK));
#endif

		return 0;
	}

	/*nomral linked list mode */
	for (i = 0; i < config->block_count; i++) {
		ll_size += sizeof(dma_linked_list_item_t);
		src = GET_LSB(block->source_address);
		dst = GET_LSB(block->dest_address);
		ctrl_low = BUILD_LL_CTRL_REG(dir, src_b,
					     dst_b, src_w, src_w,
					     block->source_addr_adj,
					     block->dest_addr_adj);
		dma_fill_linkedlist(ll_p_h, src, dst, block->block_size,
				    ctrl_low, ll_p_h->next_ll_p);
		ll_p_h = (dma_linked_list_item_t *)ll_p_h->next_ll_p;
		block = block->next_block;
	}
#if CONFIG_CACHE_FLUSHING
	sys_cache_flush((void *)*ll_p,
			ll_size + ((uint32_t)(*ll_p) & CACHE_MASK));
#else
	sedi_core_clean_dcache_by_addr((uint32_t *)*ll_p,
			ll_size + ((uint32_t)(*ll_p) & CACHE_MASK));
#endif

	return 0;
INVALID_ARGS:
	LOG_ERR("dma config failed for invalid args");
	return ret;
}
#endif

static int dma_sedi_apply_single_config(sedi_dma_t dev, uint32_t channel,
					struct dma_config *config)
{
	int ret = 0;
	uint32_t temp;

	ret = dma_sedi_apply_common_config(dev, channel, config, NULL);
	if (ret != 0) {
		goto INVALID_ARGS;
	}
	/* configurate dma width of source data*/
	ret = width_index(config->source_data_size, &temp);
	if (ret != 0) {
		goto INVALID_ARGS;
	}
	sedi_dma_control(dev, channel, SEDI_CONFIG_DMA_SR_TRANS_WIDTH, temp);

	/* configurate dma width of destination data*/
	ret = width_index(config->dest_data_size, &temp);
	if (ret != 0) {
		goto INVALID_ARGS;
	}
	sedi_dma_control(dev, channel, SEDI_CONFIG_DMA_DT_TRANS_WIDTH, temp);

	/* configurate dma burst size*/
	ret = burst_index(config->source_burst_length, &temp);
	if (ret != 0) {
		goto INVALID_ARGS;
	}
	sedi_dma_control(dev, channel, SEDI_CONFIG_DMA_BURST_LENGTH, temp);
	return 0;

INVALID_ARGS:
	LOG_ERR("dma config failed for invalid args");
	return ret;
}

static int dma_sedi_chan_config(const struct device *dev, uint32_t channel,
				struct dma_config *config)
{
	int ret = -1;

	if ((dev == NULL) || (channel >= DMA_CHANNEL_NUM) || (config == NULL)
	    || (config->polling_mode && (config->block_count != 1))) {
		goto INVALID_ARGS;
	}

	const struct dma_sedi_config_info *const info = DEV_CFG(dev);
	struct dma_sedi_driver_data *const data = DEV_DATA(dev);
	struct dma_config *local_config = &(data->dma_configs[channel]);

	memcpy(local_config, config, sizeof(struct dma_config));
	local_config->head_block =
		alloc_and_cpy_blk_nodes_slab(config->head_block);
#if CONFIG_DMA_MULTIPLE_BLOCK
	data->ll_header[channel] = NULL;
#endif

	/* initialize the dma controller, following the sedi api*/
	sedi_dma_event_cb_t cb = dma_handler;

	sedi_dma_init(info->instance, (int)channel, cb, NULL);
	return 0;

INVALID_ARGS:
	LOG_ERR("dma config failed for invalid args");
	return ret;
}

static int dma_sedi_reload(const struct device *dev, uint32_t channel,
			   uint64_t src, uint64_t dst, size_t size)
{
	if ((dev == NULL) || (channel >= DMA_CHANNEL_NUM)) {
		LOG_ERR("dma reload failed for invalid args");
		return -ENOTSUP;
	}

	int ret = 0;
	struct dma_sedi_driver_data *const data = DEV_DATA(dev);
	struct dma_config *config = &(data->dma_configs[channel]);
	struct dma_block_config *block_config = config->head_block;

	if ((config == NULL) || (block_config == NULL)) {
		LOG_ERR("dma reload failed, no config found");
		return -ENOTSUP;
	}
	if ((config->block_count == 1) || (block_config->next_block == NULL)) {
		block_config->source_address = src;
		block_config->dest_address = dst;
		block_config->block_size = size;
	} else {
		LOG_ERR("no reload support for multi-linkedlist mode");
		return -ENOTSUP;
	}
	return ret;
}

static int dma_sedi_start(const struct device *dev, uint32_t channel)
{
	if ((dev == NULL) || (channel >= DMA_CHANNEL_NUM)) {
		LOG_ERR("dma tranferring failed for invalid args");
		return -ENOTSUP;
	}

	int ret = -1;
#if CONFIG_DMA_MULTIPLE_BLOCK
	dma_linked_list_item_t *ll_p = NULL;
#endif
	const struct dma_sedi_config_info *const info = DEV_CFG(dev);
	struct dma_sedi_driver_data *const data = DEV_DATA(dev);
	struct dma_config *config = &(data->dma_configs[channel]);
	struct dma_block_config *block_config = config->head_block;
	uint64_t src_addr, dst_addr;

	if (k_is_in_isr()) {
		ret = k_sem_take(&data->sema[channel], K_NO_WAIT);
		if (ret == -EBUSY) {
			LOG_ERR("dma channel %d is busy!!!", channel);
			return -EBUSY;
		}
	} else {
		k_sem_take(&data->sema[channel], K_FOREVER);
	}

	sedi_dma_set_power(info->instance, channel, SEDI_POWER_FULL);
	if (config->block_count == 1) {
		/* call sedi start function */
		ret = dma_sedi_apply_single_config(info->instance,
						   channel, config);
		if (ret) {
			goto ERR;
		}
		src_addr = block_config->source_address;
		dst_addr = block_config->dest_address;

		if (config->polling_mode == 0) {
			ret = sedi_dma_start_transfer(info->instance, channel,
						      src_addr, dst_addr,
						      block_config->block_size);
		} else {
			ret = sedi_dma_start_transfer_polling(info->instance,
							      channel,
							      src_addr, dst_addr,
							      block_config->block_size);
		}
	} else {
#if CONFIG_DMA_MULTIPLE_BLOCK
		ret = dma_sedi_apply_ll_config(info->instance, channel, config,
					       &ll_p);
		if (ret) {
			goto ERR;
		}
		LOG_DBG("starting linked list: %p", ll_p);
		data->ll_header[channel] = ll_p;
		ret = sedi_dma_start_ll_transfer(info->instance, channel, ll_p);
#else
		LOG_ERR("MULTIPLE_BLOCK CONFIG is not set");
		goto ERR;
#endif
	}
	if (ret != SEDI_DRIVER_OK) {
		goto ERR;
	}
	return ret;

ERR:
	LOG_ERR("dma transfer failed");
	sedi_dma_set_power(info->instance, channel, SEDI_POWER_OFF);
	k_sem_give(&data->sema[channel]);
	return ret;
}

static int dma_sedi_stop(const struct device *dev, uint32_t channel)
{
	const struct dma_sedi_config_info *const info = DEV_CFG(dev);
	struct dma_sedi_driver_data *const data = DEV_DATA(dev);
	struct dma_config *config = &(data->dma_configs[channel]);

	LOG_DBG("stoping dma: %p, %d", dev, channel);
	sedi_dma_abort_transfer(info->instance, channel);
	sedi_dma_set_power(info->instance, channel, SEDI_POWER_OFF);
#if CONFIG_DMA_MULTIPLE_BLOCK
	free_ll_nodes_slab(&(data->ll_header[channel]));
#endif
	free_blk_nodes_slab(config);
	return 0;
}

static const struct dma_driver_api dma_funcs = { .config = dma_sedi_chan_config,
						 .start = dma_sedi_start,
						 .stop = dma_sedi_stop,
						 .reload = dma_sedi_reload,
						 .get_status = NULL };

static int _impl_dma_sedi_init(const struct device *dev)
{
	struct dma_sedi_driver_data *const data = DEV_DATA(dev);
	const struct dma_sedi_config_info *const info = DEV_CFG(dev);

	/* save device pointer to context*/
	dev_map[info->instance] = dev;
	/* initialize the semaphare for each channel*/
	for (int i = 0; i < DMA_CHANNEL_NUM; i++) {
		k_sem_init(&data->sema[i], 1, 1);
	}
	return 0;
}

#define DMA_OPEN_IRQ(n, sense)				    \
	IRQ_CONNECT(DT_INST_IRQN(n),			    \
			DT_INST_IRQ(n, priority),	    \
			dma_isr,			    \
			DT_INST_PROP(n, peripheral_id),	    \
			sense);				    \
	irq_enable(DT_INST_IRQN(n))

static int dma_sedi_init(const struct device *dev)
{
	const struct dma_sedi_config_info *const info = DEV_CFG(dev);
	sedi_dma_t dma_dev = info->instance;

	if (DEV_PSE_OWNED !=
	    sedi_get_dev_ownership(PSE_DEV_DMA0 + dma_dev)) {
		return -ENODEV;
	}

	_impl_dma_sedi_init(dev);

	switch (dma_dev) {

#if DT_NODE_HAS_STATUS(DT_DRV_INST(0), okay)
	case DT_INST_PROP(0, peripheral_id):
#if DT_INST_IRQ_HAS_CELL(0, sense)
		DMA_OPEN_IRQ(0, DT_INST_IRQ(0, sense));
#else
		DMA_OPEN_IRQ(0, 0);
#endif
		break;
#endif

#if DT_NODE_HAS_STATUS(DT_DRV_INST(1), okay)
	case DT_INST_PROP(1, peripheral_id):
#if DT_INST_IRQ_HAS_CELL(1, sense)
		DMA_OPEN_IRQ(1, DT_INST_IRQ(1, sense));
#else
		DMA_OPEN_IRQ(1, 0);
#endif
		break;
#endif

#if DT_NODE_HAS_STATUS(DT_DRV_INST(2), okay)
	case DT_INST_PROP(2, peripheral_id):
#if DT_INST_IRQ_HAS_CELL(2, sense)
		DMA_OPEN_IRQ(2, DT_INST_IRQ(2, sense));
#else
		DMA_OPEN_IRQ(2, 0);
#endif
		break;
#endif

	default:
		return -EIO;
	}
#if defined(CONFIG_PM_DEVICE)
	struct dma_sedi_driver_data *const data = DEV_DATA(dev);

	data->power_status = PM_DEVICE_STATE_ACTIVE;
#endif
	return 0;
}

#ifdef CONFIG_PM_DEVICE

static bool is_dma_busy(sedi_dma_t dev)
{
	sedi_dma_status_t chn_status;

	for (int chn = 0; chn < DMA_CHANNEL_NUM; chn++) {
		sedi_dma_get_status(dev, chn, &chn_status);
		if (chn_status.busy == 1) {
			return true;
		}
	}
	return false;
}

static int dma_change_device_power(const struct device *dev,
				   enum pm_device_state power_state)
{
	struct dma_sedi_driver_data *const data = DEV_DATA(dev);
	const struct dma_sedi_config_info *const info = DEV_CFG(dev);
	sedi_dma_t dma_dev = info->instance;
	int ret;

	sedi_power_state_t state;

	switch (power_state) {
	case PM_DEVICE_STATE_ACTIVE:
		state = SEDI_POWER_FULL;
		break;
	case PM_DEVICE_STATE_SUSPEND:
		if (is_dma_busy(dma_dev)) {
			return -EBUSY;
		}
		state = SEDI_POWER_SUSPEND;
		break;
	case PM_DEVICE_STATE_LOW_POWER:
		if (is_dma_busy(dma_dev)) {
			return 0;
		}
		state = SEDI_POWER_LOW;
		break;
	case PM_DEVICE_STATE_FORCE_SUSPEND:
		state = SEDI_POWER_FORCE_SUSPEND;
		break;
	default:
		return -ENOTSUP;
	}

	for (uint8_t chn = 0; chn < DMA_CHANNEL_NUM; chn++) {
		ret = sedi_dma_set_power(dma_dev, chn, state);
		if (ret != SEDI_DRIVER_OK) {
			return -EIO;
		}
	}

	data->power_status = power_state;
	return 0;
}

static int dma_sedi_device_ctrl(const struct device *dev, uint32_t ctrl_command,
				enum pm_device_state *state)
{
	int ret = 0;
	struct dma_sedi_driver_data *const data = DEV_DATA(dev);

	if (ctrl_command == PM_DEVICE_STATE_SET) {
		ret = dma_change_device_power(dev, *state);
	} else if (ctrl_command == PM_DEVICE_STATE_GET) {
		*state = data->power_status;
	}

	return ret;
}

#endif

#define DMA_SEDI_DEVICE_INIT(n)						\
	struct dma_sedi_driver_data dma_sedi_dev_data_##n;	\
	const struct dma_sedi_config_info dma_sedi_config_data_##n = {\
		.instance = DT_INST_PROP(n, peripheral_id),		 \
	};								\
	DEVICE_DEFINE(dma_sedi_##n, DT_LABEL(DT_DRV_INST(n)), &dma_sedi_init,\
			dma_sedi_device_ctrl,				\
			&dma_sedi_dev_data_##n, &dma_sedi_config_data_##n,	\
			PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,\
			(void *)&dma_funcs);

DT_INST_FOREACH_STATUS_OKAY(DMA_SEDI_DEVICE_INIT)
