/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT intel_pse_i2s

#include <errno.h>
#include <string.h>
#include <sys/__assert.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/dma.h>
#include <drivers/i2s.h>
#include <soc.h>
#include <sedi.h>
#include "driver/sedi_driver_i2s.h"
#include "i2s_sedi.h"


#define LOG_LEVEL CONFIG_SYS_LOG_I2S_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(i2s_sedi);

#define I2S_LOG_ERR LOG_ERR
#define I2S_LOG_WRN LOG_WRN
#define I2S_LOG_INFO LOG_INF
#define I2S_LOG_TRACE LOG_DBG

#define DMA_HWID_I2S0_INSTANCE DMA_HWID_I2S0
#define DMA_HWID_I2S1_INSTANCE DMA_HWID_I2S1

#define EN_I2S_FULL_DUPLEX (1)

#if defined(CONFIG_DCACHE_I2S_DMA_WRITEBACK) && !defined(CONFIG_CACHE_DISABLE)
#define DCACHE_INVALIDATE(addr, size)			  \
	{						  \
		SCB_InvalidateDCache_by_Addr(addr, size); \
	}
#define DCACHE_CLEAN(addr, size)		     \
	{					     \
		SCB_CleanDCache_by_Addr(addr, size); \
	}
#else
#define DCACHE_INVALIDATE(addr, size) \
	do {			      \
	} while (0)

#define DCACHE_CLEAN(addr, size) \
	do {			 \
	} while (0)
#endif

#define SEDI_SSP_WORD_SIZE_BITS_MIN 8
#define SEDI_SSP_WORD_SIZE_BITS_MAX 32
#define SEDI_SSP_WORD_PER_FRAME_MIN 1
#define SEDI_SSP_WORD_PER_FRAME_MAX 8

#define TX_STRM_ACTIVE BIT(0)
#define RX_STRM_ACTIVE BIT(1)

struct i2s_sedi_dev_data;

struct queue_item {
	void *mem_block;
	size_t size;
};

/* Minimal ring buffer implementation:
 * buf - holds the pointer to Queue items
 * len - Max number of Queue items that can be referenced
 * head - current write index number
 * tail - current read index number
 */
struct ring_buf {
	struct queue_item *buf;
	uint16_t len;
	uint16_t head;
	uint16_t tail;
};

/* This indicates the Tx/Rx stream. Most members of the stream are
 * self-explanatory except for sem.
 * sem - This is initialized to CONFIG_I2S_SEDI_TX_BLOCK_COUNT. If at all
 * mem_block_queue gets filled with the MAX blocks configured, this semaphore
 * ensures nothing gets written into the mem_block_queue before a slot gets
 * freed (which happens when a block gets read out).
 */
struct stream {
	int32_t state;
	struct k_sem sem;
#if ZEPHYR_DMA_DRV
	uint32_t dma_channel;
	struct dma_config dma_cfg;
#else
	sedi_i2s_dma_config_t dma_cfg;
#endif
	struct i2s_config cfg;
	struct ring_buf mem_block_queue;
	void *mem_block;
	bool last_block;
	uint8_t fifo_indx;
	int32_t hw_fifo;
#if defined(CONFIG_I2S_TIMESTAMP_EN)
	uint32_t start_time;
	uint32_t stop_time;
#endif
	struct k_sem dma_stop_alert;
	int (*stream_start)(struct stream *stream, struct i2s_sedi_dev_data *dev_data,
			    struct device *dev);
	void (*stream_disable)(struct stream *stream, struct i2s_sedi_dev_data *dev_data,
			       struct device *dev);
	void (*queue_drop)(struct stream *stream);
};

struct i2s_sedi_config {
	struct i2s_sedi_ssp *regs;
	struct i2s_sedi_mn_div *mn_regs;
	uint32_t irq_id;
	void (*irq_config)(void);
};

/* Device run time data */
struct i2s_sedi_dev_data {
	int dev_instance : 4;
	int i2s_mode : 2;
	int stream_active : 2;
	struct device *dev_dma;
	struct stream tx;
	struct stream rx;
};

/* DMA context */
struct i2s_dma_ctx_t {
	uint32_t tx_chnl : 4;
	uint32_t rx_chnl : 4;
	struct device *dev;
};

#define DEV_NAME(dev) ((dev)->name)
#define DEV_CFG(dev) \
	((const struct i2s_sedi_config *const)(dev)->config)
#define DEV_DATA(dev) ((struct i2s_sedi_dev_data *const)(dev)->data)

static void tx_stream_disable(struct stream *,
			      struct i2s_sedi_dev_data *dev_data,
			      struct device *);
static void rx_stream_disable(struct stream *stream,
			      struct i2s_sedi_dev_data *dev_data,
			      struct device *dev_dma);
static int rx_stream_start(struct stream *stream,
			   struct i2s_sedi_dev_data *dev_data,
			   struct device *dev_dma);

#if defined(CONFIG_I2S_TIMESTAMP_EN)
static void init_time_stamp(struct stream *strm)
{
	strm->start_time = k_cycle_get_32();
}

static void print_time_stamp(char *msg, struct stream *strm)
{
	uint32_t cycles_spent;
	uint32_t nanoseconds_spent, time;

	time = k_cycle_get_32();
	if (time > strm->start_time) {
		cycles_spent = time - strm->start_time;
	} else {
		cycles_spent = time + (0xFFFFFFFF - strm->start_time);
	}
	nanoseconds_spent = SYS_CLOCK_HW_CYCLES_TO_NS(cycles_spent) / 5;

	I2S_PRINT("%s %d nS\n", msg, nanoseconds_spent);

	strm->start_time = k_cycle_get_32();
}
#else
#define init_time_stamp(...)
#define print_time_stamp(...)
#endif

#if ZEPHYR_DMA_DRV
static struct device *get_dev_from_dma_channel(uint32_t dma_channel)
{
	I2S_LOG_TRACE("%s\n", __func__);
	int i;
	static struct i2s_dma_ctx_t i2s_dma_ctx[MAX_I2S_INSTANCE];

	for (i = 0; i < MAX_I2S_INSTANCE; i++) {
		if ((dma_channel == i2s_dma_ctx[i].tx_chnl) ||
		    (dma_channel == i2s_dma_ctx[i].rx_chnl)) {
			return i2s_dma_ctx[i].dev;
		}
	}

	return NULL;
}
#endif

static inline uint16_t modulo_inc(uint16_t val, uint16_t max)
{
	val++;
	return (val < max) ? val : 0;
}

/*
 * Get data from the queue
 */
static int queue_get(struct ring_buf *rb, uint8_t mode, void **mem_block,
		     size_t *size)
{
	I2S_LOG_TRACE("%s\n", __func__);
	unsigned int key;

	key = irq_lock();

	/* In case of ping-pong mode, the buffers are not freed after
	 * reading. They are fixed in size. Another thread will populate
	 * the pong buffer while the ping buffer is being read out and
	 * vice versa. Hence, we just need to keep reading from buffer0
	 * (ping buffer) followed by buffer1 (pong buffer) and the same
	 * cycle continues.
	 *
	 * In case of non-ping-pong modes, each buffer is freed after it
	 * is read. The tail pointer will keep progressing depending upon
	 * the reads. The head pointer will move whenever there's a write.
	 * If tail equals head, it would mean we have read everything there
	 * is and the buffer is empty.
	 */
	if (rb->tail == rb->head) {
		if ((mode & I2S_OPT_PINGPONG) == I2S_OPT_PINGPONG) {
			/* Point back to the first element */
			rb->tail = 0;
		} else {
			/* Ring buffer is empty */
			I2S_LOG_INFO("Ring buffer is empty\n");
			irq_unlock(key);
			return -ENOMEM;
		}
	}

	*mem_block = rb->buf[rb->tail].mem_block;
	*size = rb->buf[rb->tail].size;
	rb->tail = modulo_inc(rb->tail, rb->len);

	irq_unlock(key);

	return 0;
}

/*
 * Put data in the queue
 */
static int queue_put(struct ring_buf *rb, uint8_t mode, void *mem_block,
		     size_t size)
{
	I2S_LOG_TRACE("%s\n", __func__);

	uint16_t head_next;
	unsigned int key;

	key = irq_lock();

	head_next = rb->head;
	head_next = modulo_inc(head_next, rb->len);

	/* In case of ping-pong mode, the below comparison incorrectly
	 * leads to complications as the buffers are always predefined.
	 * Hence excluding ping-pong mode from this comparison.
	 */
	if ((mode & I2S_OPT_PINGPONG) != I2S_OPT_PINGPONG) {
		if (head_next == rb->tail) {
			/* Ring buffer is full */
			irq_unlock(key);
			I2S_LOG_INFO("Ring buffer is empty\n");
			return -ENOMEM;
		}
	}

	rb->buf[rb->head].mem_block = mem_block;
	rb->buf[rb->head].size = size;
	rb->head = head_next;

	irq_unlock(key);

	return 0;
}

#if ZEPHYR_DMA_DRV
static int start_dma(struct i2s_sedi_dev_data *dev_data, uint32_t channel,
		     struct dma_config *cfg, void *src, void *dst,
		     uint32_t blk_size)
{
	I2S_LOG_TRACE("%s\n", __func__);
	int ret;
	struct device *dev_dma = dev_data->dev_dma;

	struct dma_block_config blk_cfg = {
		.block_size = blk_size,
		.source_address = (uint32_t)src,
		.dest_address = (uint32_t)dst,
	};

	cfg->head_block = &blk_cfg;

	ret = dma_config(dev_dma, channel, cfg);
	if (ret < 0) {
		I2S_LOG_ERR("dma_config failed: %d", ret);
		return ret;
	}

	if (cfg->channel_direction == DMA_PERIPHERAL_TO_MEMORY) {
		sedi_i2s_reset_rx_fifo(dev_data->dev_instance);
	}
	ret = dma_start(dev_dma, channel);
	if (ret < 0) {
		I2S_LOG_ERR("dma_start failed: %d", ret);
	}

	return ret;
}
#else
static int start_dma(struct i2s_sedi_dev_data *dev_data,
		     sedi_dma_t dma_device, sedi_i2s_dma_config_t *cfg,
		     void *src, void *dst, uint32_t blk_size)
{
	I2S_LOG_TRACE("%s\n", __func__);
	int ret;
	unsigned int key;

	sedi_i2s_dma_add_t blk_cfg = {
		.channel = cfg->channel,
		.block_size = blk_size,
		.source_address = (uint32_t)src,
		.dest_address = (uint32_t)dst,
	};

	key = irq_lock();

	ret = sedi_i2s_config_dma(dma_device, cfg->channel_direction, cfg);
	if (ret != 0) {
		I2S_LOG_ERR("sedi_i2s_config_dma failed: %d", ret);
	} else {
		if (cfg->channel_direction == DMA_PERIPHERAL_TO_MEMORY) {
			sedi_i2s_reset_rx_fifo(dev_data->dev_instance);
		}
		ret = sedi_i2s_dma_start(dma_device, &blk_cfg);
	}

	irq_unlock(key);

	return ret;
}
#endif

/* This function is executed in the interrupt context */
#if ZEPHYR_DMA_DRV
static void dma_rx_callback(struct device *dev_dma, uint32_t channel, int status)
{
	struct device *dev = get_dev_from_dma_channel(channel);
	struct i2s_sedi_dev_data *const dev_data = DEV_DATA(dev);
	struct stream *stream = &dev_data->rx;
	int ret;

#else
static void dma_rx_callback(sedi_dma_t dma_device, int channel_id, int event,
			    void *ctx)
{
	struct i2s_sedi_dev_data *const dev_data =
		(struct i2s_sedi_dev_data *const)ctx;
	struct stream *stream = &dev_data->rx;
	int ret;

	if (event != SEDI_DMA_EVENT_TRANSFER_DONE) {
		I2S_LOG_ERR("%s: Error in DMA Event\n", __func__);
		goto rx_disable;
	}
#endif
	I2S_LOG_TRACE("%s\n", __func__);
	print_time_stamp("rx cb:", stream);

	__ASSERT_NO_MSG(stream->mem_block != NULL);

	/* Stop reception if there was an error */
	if (stream->state == I2S_STATE_ERROR) {
		goto rx_disable;
	}

	/* Assure cache coherency after DMA write operation */
	DCACHE_INVALIDATE(stream->mem_block, stream->cfg.block_size);

	/* All block data received */
	ret = queue_put(&stream->mem_block_queue, stream->cfg.options,
			stream->mem_block, stream->cfg.block_size);
	if (ret < 0) {
		stream->state = I2S_STATE_ERROR;
		goto rx_disable;
	}

	stream->mem_block = NULL;
	k_sem_give(&stream->sem);

	/* Stop reception if we were requested */
	if (stream->state == I2S_STATE_STOPPING) {
		stream->state = I2S_STATE_READY;
		I2S_LOG_ERR("Stream state != I2S_STATE_STOPPING and so stopping capture!\n");
		goto rx_disable;
	}

	/* Prepare to receive the next data block */
	ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block,
			       K_NO_WAIT);
	if (ret < 0) {
		I2S_LOG_ERR("RX memory slab allocate failed!\n");
		stream->state = I2S_STATE_ERROR;
		goto rx_disable;
	}
#if ZEPHYR_DMA_DRV
	ret = start_dma(dev_data, stream->dma_channel, &stream->dma_cfg,
			(void *)stream->hw_fifo, stream->mem_block,
			stream->cfg.block_size);
	if (ret < 0) {
		I2S_LOG_ERR("Failed to start RX DMA transfer: %d", ret);
		goto rx_disable;
	}
#else
	ret = start_dma(dev_data, dma_device, &stream->dma_cfg,
			(void *)stream->hw_fifo, stream->mem_block,
			stream->cfg.block_size);
	if (ret != 0) {
		I2S_LOG_ERR("Failed to start RX DMA transfer: %d", ret);
		goto rx_disable;
	}
#endif
	return;

rx_disable:
	I2S_LOG_ERR("Disabling RX Path\n");
	k_sem_give(&stream->dma_stop_alert);
	rx_stream_disable(stream, dev_data, dev_data->dev_dma);
}

/* This function is executed in the interrupt context */
#if ZEPHYR_DMA_DRV
static void dma_tx_callback(struct device *dev_dma, uint32_t channel, int status)
{
	struct device *dev = get_dev_from_dma_channel(channel);
	const struct i2s_sedi_config *const dev_cfg = DEV_CFG(dev);
	struct i2s_sedi_dev_data *const dev_data = DEV_DATA(dev);
	struct stream *strm = &dev_data->tx;
	size_t mem_block_size;
	int ret;

#else
static void dma_tx_callback(sedi_dma_t dma_device, int channel_id, int event,
			    void *ctx)
{
	struct i2s_sedi_dev_data *const dev_data = ctx;
	struct stream *strm = &dev_data->tx;
	size_t mem_block_size;
	int ret;

#endif

	I2S_LOG_TRACE("%s\n", __func__);

	print_time_stamp("tx cb:", strm);

	__ASSERT_NO_MSG(strm->mem_block != NULL);

	if ((strm->cfg.options & I2S_OPT_PINGPONG) != I2S_OPT_PINGPONG) {
		/* All block data sent */
		k_mem_slab_free(strm->cfg.mem_slab, &strm->mem_block);
		strm->mem_block = NULL;
	}

	/* Stop transmission if there was an error */
	if (strm->state == I2S_STATE_ERROR) {
		I2S_LOG_ERR("TX error detected");
		goto tx_disable;
	}

	/* Stop transmission if we were requested */
	if (strm->last_block) {
		I2S_LOG_TRACE("Last Block request, Stopping...\n");
		strm->state = I2S_STATE_READY;
		goto tx_disable;
	}

	/* Prepare to send the next data block */
	ret = queue_get(&strm->mem_block_queue, strm->cfg.options,
			&strm->mem_block, &mem_block_size);
	if (ret < 0) {
		I2S_LOG_ERR("No More Data to Send, Stopping Tx..\n");
		if (strm->state == I2S_STATE_STOPPING) {
			strm->state = I2S_STATE_READY;
		} else {
			strm->state = I2S_STATE_ERROR;
		}
		goto tx_disable;
	}

	/* Assure cache coherency before DMA read operation */
	DCACHE_CLEAN(strm->mem_block, mem_block_size);

#if ZEPHYR_DMA_DRV
	ret = start_dma(dev_data, strm->dma_channel, &strm->dma_cfg,
			strm->mem_block, (void *)strm->hw_fifo, mem_block_size);
	if (ret < 0) {
		I2S_LOG_ERR("Failed to start TX DMA transfer: %d", ret);
		k_sem_give(&strm->sem);
		goto tx_disable;
	}
#else
	ret = start_dma(dev_data, dma_device, &strm->dma_cfg, strm->mem_block,
			(void *)strm->hw_fifo, strm->cfg.block_size);
	if (ret != 0) {
		I2S_LOG_ERR("Failed to start RX DMA transfer: %d", ret);
		goto tx_disable;
	}

#endif
	k_sem_give(&strm->sem);
	return;

tx_disable:
	I2S_LOG_TRACE("Disable TX Path\n");
	k_sem_give(&strm->dma_stop_alert);
	k_sem_give(&strm->sem);
	tx_stream_disable(strm, dev_data, dev_data->dev_dma);
}

static int i2s_sedi_configure(const struct device *dev, enum i2s_dir dir,
			      const struct i2s_config *i2s_cfg)
{
	I2S_LOG_TRACE("%s\n", __func__);
	struct i2s_sedi_dev_data *const dev_data = DEV_DATA(dev);
	uint8_t num_words = i2s_cfg->channels;
	uint8_t word_size_bits = i2s_cfg->word_size;
	struct stream *strm;
	i2s_config_t i2s_hw_cfg;
	uint8_t word_size_bytes;

	i2s_hw_cfg.tx_dn = 0;
	i2s_hw_cfg.rx_dn = 0;

	if (dir == I2S_DIR_TX) {
		strm = &dev_data->tx;
	} else if (dir == I2S_DIR_RX) {
		strm = &dev_data->rx;
	} else {
		I2S_LOG_ERR(" Direction must be selected.");
		return -EINVAL;
	}

	if (strm->state != I2S_STATE_NOT_READY &&
	    strm->state != I2S_STATE_READY) {
		I2S_LOG_ERR("invalid state");
		return -EINVAL;
	}

	if (i2s_cfg->frame_clk_freq == 0) {
		strm->queue_drop(strm);
		(void)memset(&strm->cfg, 0, sizeof(struct i2s_config));
		strm->state = I2S_STATE_NOT_READY;
		return 0;
	}

	if (word_size_bits < SEDI_SSP_WORD_SIZE_BITS_MIN ||
	    word_size_bits > SEDI_SSP_WORD_SIZE_BITS_MAX) {
		I2S_LOG_ERR("Unsupported I2S word size");
		return -EINVAL;
	}

	if (num_words < SEDI_SSP_WORD_PER_FRAME_MIN ||
	    num_words > SEDI_SSP_WORD_PER_FRAME_MAX) {
		I2S_LOG_ERR("Unsupported words per frame number");
		return -EINVAL;
	}

	memcpy(&strm->cfg, i2s_cfg, sizeof(struct i2s_config));

	i2s_hw_cfg.rx_bits = 0;

	/* set mode as slave or master*/
	if (i2s_cfg->options &
	    (I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE)) {
		if (dev_data->i2s_mode == EN_I2S_FULL_DUPLEX) {
			i2s_hw_cfg.mode = I2S_SLAVE_FULL_DUPLEX;
		} else {
			i2s_hw_cfg.mode = (dir == I2S_DIR_TX) ?
					  I2S_SLAVE_TX : I2S_SLAVE_RX;
		}
	} else {
		if (dev_data->i2s_mode == EN_I2S_FULL_DUPLEX) {
			i2s_hw_cfg.mode = I2S_MASTER_FULL_DUPLEX;
		} else {
			i2s_hw_cfg.mode = (dir == I2S_DIR_TX) ?
					  I2S_MASTER_TX : I2S_MASTER_RX;
		}
	}

	/* clock signal polarity */
	switch (i2s_cfg->format & I2S_FMT_CLK_FORMAT_MASK) {
	case I2S_FMT_CLK_NF_NB:
		i2s_hw_cfg.polariy = I2S_CLK_NF_NB;
		break;

	case I2S_FMT_CLK_NF_IB:
		i2s_hw_cfg.polariy = I2S_CLK_NF_IB;
		break;

	case I2S_FMT_CLK_IF_NB:
		i2s_hw_cfg.polariy = I2S_CLK_IF_NB;
		break;

	case I2S_FMT_CLK_IF_IB:
		i2s_hw_cfg.polariy = I2S_CLK_IF_IB;
		break;

	default:
		I2S_LOG_ERR("Unsupported Clock format");
		return -EINVAL;
	}

	/* Configure I2S format */
	i2s_hw_cfg.bits = word_size_bits;
	if (dir == I2S_DIR_RX) {
		i2s_hw_cfg.rx_bits =
			word_size_bits;
	}
	i2s_hw_cfg.channel = num_words;
	i2s_hw_cfg.sample_rate = i2s_cfg->frame_clk_freq;

	if (num_words > 2) {
		i2s_hw_cfg.tdm_ch.bits = num_words;
		i2s_hw_cfg.tdm_tx_ch.bits = (0xFFFF >> num_words);
		i2s_hw_cfg.tdm_rx_ch.bits = (0xFFFF >> num_words);
	}

	if (i2s_cfg->format & I2S_FMT_DATA_ORDER_LSB) {
		i2s_hw_cfg.msb_first = 0;
	} else {
		i2s_hw_cfg.msb_first = 1;
	}

	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {

	case I2S_FMT_DATA_FORMAT_I2S:
		i2s_hw_cfg.channel_format = I2S_CHANNEL_FMT_STD;
		break;

	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		i2s_hw_cfg.channel_format = I2S_CHANNEL_FMT_LEFT;
		i2s_hw_cfg.polariy = I2S_CLK_IF_NB;
		break;
	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		i2s_hw_cfg.channel_format = I2S_CHANNEL_FMT_RIGHT;
		i2s_hw_cfg.polariy = I2S_CLK_IF_NB;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
		i2s_hw_cfg.channel_format = I2S_CHANNEL_FMT_TDM;
		i2s_hw_cfg.polariy = I2S_CLK_NF_IB;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_LONG:
	default:
		I2S_LOG_ERR("Unsupported I2S data format");
		return -EINVAL;
	}

	if (sedi_i2s_init(dev_data->dev_instance, &i2s_hw_cfg) !=
	    SEDI_DRIVER_OK) {
		return -EIO;
	}

	/* Set up DMA channel parameters */
	switch (word_size_bits) {
	case BIT_WORD_LEN_12:
	case BIT_WORD_LEN_16:
		word_size_bytes = I2S_DATA_WIDTH_2B;
		break;

	case BIT_WORD_LEN_24:
	case BIT_WORD_LEN_32:
		word_size_bytes = I2S_DATA_WIDTH_4B;
		break;

	default:
		I2S_LOG_ERR("Unsupported I2S data format: Bit per sample !!");
		return -EINVAL;
	}

	strm->dma_cfg.source_data_size = word_size_bytes;
	strm->dma_cfg.dest_data_size = word_size_bytes;

	strm->state = I2S_STATE_READY;
	return 0;
}

static const struct i2s_config *i2s_sedi_config_get(const struct device *dev,
						    enum i2s_dir dir)
{
	I2S_LOG_TRACE("%s\n", __func__);
	struct i2s_sedi_dev_data *const dev_data = DEV_DATA(dev);
	struct stream *stream;

	if (dir == I2S_DIR_RX) {
		stream = &dev_data->rx;
	} else {
		stream = &dev_data->tx;
	}

	if (stream->state == I2S_STATE_NOT_READY) {
		return NULL;
	}

	return &stream->cfg;
}


static int rx_stream_start(struct stream *stream,
			   struct i2s_sedi_dev_data *dev_data,
			   struct device *dev_dma)
{
	I2S_LOG_TRACE("%s\n", __func__);
#if !defined(CONFIG_EN_I2S_POLLED_IO)
	int ret;

	ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block,
			       K_NO_WAIT);
	if (ret < 0) {
		return ret;
	}
#if ZEPHYR_DMA_DRV
	ret = start_dma(dev_data, stream->dma_channel, &stream->dma_cfg,
			(void *)stream->hw_fifo, stream->mem_block,
			stream->cfg.block_size);
	if (ret < 0) {
		I2S_LOG_ERR("Failed to start RX DMA transfer: %d", ret);
		return ret;
	}
#else
	ret = start_dma(dev_data, I2S_DMA_INSTANCE, &stream->dma_cfg,
			(void *)stream->hw_fifo, stream->mem_block,
			stream->cfg.block_size);
	if (ret < 0) {
		I2S_LOG_ERR("Failed to start RX DMA transfer: %d", ret);
		return ret;
	}
#endif

	stream->fifo_indx = 0;
#endif
	/* Enable port. */
	sedi_enable_transceiver(dev_data->dev_instance);
	sedi_i2s_enable_interrupt(dev_data->dev_instance);

	dev_data->stream_active |= RX_STRM_ACTIVE;

	return 0;
}

static int tx_stream_start(struct stream *strm,
			   struct i2s_sedi_dev_data *dev_data,
			   struct device *dev_dma)
{
	I2S_LOG_TRACE("%s\n", __func__);
	size_t mem_block_size;
	int ret;

	ret = queue_get(&strm->mem_block_queue, strm->cfg.options,
			&strm->mem_block, &mem_block_size);
	if (ret < 0) {
		I2S_LOG_ERR("TX Queue is empty\n");
		return ret;
	}

	strm->fifo_indx = 0;

	k_sem_give(&strm->sem);
#if !defined(CONFIG_EN_I2S_POLLED_IO)
	/* Assure cache coherency before DMA read operation */
	DCACHE_CLEAN(strm->mem_block, mem_block_size);

#if ZEPHYR_DMA_DRV
	ret = start_dma(dev_data, strm->dma_channel, &strm->dma_cfg,
			strm->mem_block, (void *)strm->hw_fifo, mem_block_size);
	if (ret < 0) {
		I2S_LOG_ERR("Failed to start TX DMA transfer: %d", ret);
		return ret;
	}
#else
	ret = start_dma(dev_data, I2S_DMA_INSTANCE, &strm->dma_cfg,
			strm->mem_block, (void *)strm->hw_fifo, mem_block_size);
	if (ret < 0) {
		I2S_LOG_ERR("Failed to start TX DMA transfer: %d", ret);
		return ret;
	}
#endif
#endif
	/* enable port */
	sedi_enable_transceiver(dev_data->dev_instance);
	sedi_i2s_enable_interrupt(dev_data->dev_instance);

	dev_data->stream_active |= TX_STRM_ACTIVE;

	init_time_stamp(strm);

	return 0;
}

static void rx_stream_disable(struct stream *stream,
			      struct i2s_sedi_dev_data *dev_data,
			      struct device *dev_dma)
{
	I2S_LOG_TRACE("%s\n", __func__);
#if !defined(CONFIG_EN_I2S_POLLED_IO)
	unsigned int key;
#endif

	/* Check other active tx stream before disable. */
	if (dev_data->i2s_mode == I2S_EN_FULLDUPLEX_MODE) {
		if (!(dev_data->stream_active & TX_STRM_ACTIVE)) {
			sedi_i2s_disable_interrupt(dev_data->dev_instance);
			sedi_disable_transceiver(dev_data->dev_instance);
		}
	} else {
		sedi_i2s_disable_interrupt(dev_data->dev_instance);
		sedi_disable_transceiver(dev_data->dev_instance);
	}
#if !defined(CONFIG_EN_I2S_POLLED_IO)
#if ZEPHYR_DMA_DRV
	dma_stop(dev_dma, stream->dma_channel);
#else
	key = irq_lock();
	sedi_i2s_dma_stop(I2S_DMA_INSTANCE, stream->dma_cfg.channel);
	irq_unlock(key);
#endif
	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, &stream->mem_block);
		stream->mem_block = NULL;
	}

#endif
	dev_data->stream_active &= ~RX_STRM_ACTIVE;
}

static void tx_stream_disable(struct stream *strm,
			      struct i2s_sedi_dev_data *dev_data,
			      struct device *dev_dma)
{
	I2S_LOG_TRACE("%s\n", __func__);
#if !defined(CONFIG_EN_I2S_POLLED_IO)
	unsigned int key;
#endif

	/* Check other active rx stream before disable. */
	if (dev_data->i2s_mode == I2S_EN_FULLDUPLEX_MODE) {
		if (!(dev_data->stream_active & RX_STRM_ACTIVE)) {
			sedi_i2s_disable_interrupt(dev_data->dev_instance);
			sedi_disable_transceiver(dev_data->dev_instance);
		}
	} else {
		sedi_i2s_disable_interrupt(dev_data->dev_instance);
		sedi_disable_transceiver(dev_data->dev_instance);
	}

#if !defined(CONFIG_EN_I2S_POLLED_IO)
#if ZEPHYR_DMA_DRV
	dma_stop(dev_dma, strm->dma_channel);
#else
	key = irq_lock();
	sedi_i2s_dma_stop(I2S_DMA_INSTANCE, strm->dma_cfg.channel);
	irq_unlock(key);
#endif
#endif
	if (((strm->cfg.options & I2S_OPT_PINGPONG) != I2S_OPT_PINGPONG) &&
	    (strm->mem_block != NULL)) {
		k_mem_slab_free(strm->cfg.mem_slab, &strm->mem_block);
		strm->mem_block = NULL;
		strm->mem_block_queue.head = 0;
		strm->mem_block_queue.tail = 0;
	}

	dev_data->stream_active &= ~TX_STRM_ACTIVE;
}

static void rx_queue_drop(struct stream *stream)
{
	I2S_LOG_TRACE("%s\n", __func__);
	size_t size;
	void *mem_block;

	while (queue_get(&stream->mem_block_queue, stream->cfg.options,
			 &mem_block, &size) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, &mem_block);
	}

	k_sem_reset(&stream->sem);
}

static void tx_queue_drop(struct stream *strm)
{
	I2S_LOG_TRACE("%s\n", __func__);
	size_t size;
	void *mem_block;
	unsigned int n = 0;

	while (queue_get(&strm->mem_block_queue,
			 strm->cfg.options, &mem_block, &size) == 0) {
		if ((strm->cfg.options & I2S_OPT_PINGPONG) !=
		    I2S_OPT_PINGPONG) {
			k_mem_slab_free(strm->cfg.mem_slab, &mem_block);
		} else {
			break;
		}
		n++;
	}

	if (n) {
		strm->mem_block_queue.head = 0;
		strm->mem_block_queue.tail = 0;
	}

	n = CONFIG_I2S_SEDI_TX_BLOCK_COUNT;
	for (; n > 0; n--) {
		k_sem_give(&strm->sem);
	}
}

static int i2s_sedi_trigger(const struct device *dev, enum i2s_dir dir,
			    enum i2s_trigger_cmd cmd)
{
	I2S_LOG_TRACE("%s\n", __func__);
	struct i2s_sedi_dev_data *const dev_data = DEV_DATA(dev);
	struct stream *strm;
	unsigned int key;
#if !defined(CONFIG_EN_I2S_POLLED_IO)
	int ret;
#endif

	strm = (dir == I2S_DIR_TX) ? &dev_data->tx : &dev_data->rx;

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (strm->state != I2S_STATE_READY) {
			I2S_LOG_ERR("START trigger: invalid state");
			return -EIO;
		}
		if ((strm->cfg.options & I2S_OPT_PINGPONG) !=
		    I2S_OPT_PINGPONG) {
			__ASSERT_NO_MSG(strm->mem_block == NULL);
		}
#if !defined(CONFIG_EN_I2S_POLLED_IO)
		ret = strm->stream_start(strm, dev_data, dev_data->dev_dma);
		if (ret < 0) {
			I2S_LOG_ERR("START trigger failed %d", ret);
			return ret;
		}
#else
		sedi_enable_transceiver(dev_data->dev_instance);
#endif
		strm->state = I2S_STATE_RUNNING;
		strm->last_block = false;
		break;

	case I2S_TRIGGER_STOP:
		I2S_LOG_TRACE("I2S_TRIGGER_STOP\n");
		key = irq_lock();
		if (strm->state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			I2S_LOG_ERR("STOP trigger: invalid state");
			return -EIO;
		}
		strm->state = I2S_STATE_STOPPING;
		irq_unlock(key);
		strm->last_block = true;
#if !defined(CONFIG_EN_I2S_POLLED_IO)
		/* Wait for last DMA transfer to complete. */
		ret = k_sem_take(&strm->dma_stop_alert, SYS_TIMEOUT_MS(strm->cfg.timeout));
		if (ret != 0) {
			return ret;
		}
#else
		sedi_disable_transceiver(dev_data->dev_instance);
#endif
		break;

	case I2S_TRIGGER_DRAIN:
		key = irq_lock();
		if (strm->state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			I2S_LOG_ERR("DRAIN trigger: invalid state");
			return -EIO;
		}
		strm->state = I2S_STATE_STOPPING;
		irq_unlock(key);
		break;

	case I2S_TRIGGER_DROP:
		if (strm->state == I2S_STATE_NOT_READY) {
			I2S_LOG_ERR("DROP trigger: invalid state");
			return -EIO;
		}
#if !defined(CONFIG_EN_I2S_POLLED_IO)
		if (strm->state == I2S_STATE_RUNNING) {
			strm->stream_disable(strm, dev_data, dev_data->dev_dma);
		}
		strm->queue_drop(strm);
#else
		sedi_disable_transceiver(dev_data->dev_instance);
#endif
		strm->state = I2S_STATE_READY;
		break;

	case I2S_TRIGGER_PREPARE:
		if (strm->state != I2S_STATE_ERROR) {
			I2S_LOG_ERR("PREPARE trigger: invalid state");
			return -EIO;
		}
		strm->state = I2S_STATE_READY;
		strm->queue_drop(strm);
		break;

	default:
		I2S_LOG_ERR("Unsupported trigger command");
		return -EINVAL;
	}

	return 0;
}

static int i2s_sedi_read(const struct device *dev, void **mem_block, size_t *size)
{
	I2S_LOG_TRACE("%s\n", __func__);
	struct i2s_sedi_dev_data *const dev_data = DEV_DATA(dev);
	struct stream *strm = &dev_data->rx;
	int ret;

	if (strm->state != I2S_STATE_RUNNING &&
	    strm->state != I2S_STATE_READY) {
		I2S_LOG_ERR("invalid state");
		return -EIO;
	}

	__ASSERT_NO_MSG(mem_block != NULL && size != NULL);

#if defined(CONFIG_EN_I2S_POLLED_IO)
	*mem_block = dev_data->rx.mem_block_queue.buf;
	ret = sedi_i2s_poll_read(dev_data->dev_instance, *mem_block,
				 *size);
	if (ret != SEDI_DRIVER_OK) {
		return -EIO;
	}
#else
	ret = k_sem_take(&dev_data->rx.sem, SYS_TIMEOUT_MS(dev_data->rx.cfg.timeout));
	if (ret < 0) {
		I2S_LOG_ERR("Failure taking sem");
		return ret;
	}

	/* Get data from RX queue */
	queue_get(&dev_data->rx.mem_block_queue, strm->cfg.options, mem_block,
		  size);

#endif
	return 0;
}
static int i2s_sedi_write(const struct device *dev, void *mem_block, size_t size)
{
	I2S_LOG_TRACE("%s\n", __func__);

	struct i2s_sedi_dev_data *const dev_data = DEV_DATA(dev);
	struct stream *strm = &dev_data->tx;
	int ret;

	if (strm->state != I2S_STATE_RUNNING &&
	    strm->state != I2S_STATE_READY) {
		I2S_LOG_ERR("invalid state");
		return -EIO;
	}

	__ASSERT_NO_MSG(mem_block != NULL && size != 0);

#if !defined(CONFIG_EN_I2S_POLLED_IO)
	ret = k_sem_take(&dev_data->tx.sem, SYS_TIMEOUT_MS(dev_data->tx.cfg.timeout));
	if (ret < 0) {
		I2S_LOG_ERR("Failure taking sem");
		return ret;
	}

	/* Add data to the end of the TX queue */
	queue_put(&dev_data->tx.mem_block_queue, strm->cfg.options, mem_block,
		  size);
#else
	ret = sedi_i2s_poll_write(dev_data->dev_instance, mem_block, size);
	if (ret != SEDI_DRIVER_OK) {
		return -EIO;
	}
#endif

	return 0;
}

/* Interrupt handler */
static void i2s_sedi_isr(void *arg)
{
	I2S_LOG_TRACE("%s\n", __func__);
	struct device *dev = (struct device *)arg;
	struct i2s_sedi_dev_data *const dev_data = DEV_DATA(dev);

	sedi_i2s_disable_interrupt(dev_data->dev_instance);

	sedi_interrupt_clear(dev_data->dev_instance);
	sedi_i2s_enable_interrupt(dev_data->dev_instance);
}

static int i2s_sedi_initialize(const struct device *dev)
{
	I2S_LOG_TRACE("%s\n", __func__);
	const struct i2s_sedi_config *const dev_cfg = DEV_CFG(dev);
	struct i2s_sedi_dev_data *const dev_data = DEV_DATA(dev);
	struct stream *strm_tx = &dev_data->tx;
	struct stream *strm_rx = &dev_data->rx;

	if (DEV_PSE_OWNED !=
	    sedi_get_dev_ownership(PSE_DEV_I2S0 + dev_data->dev_instance)) {
		return -ENODEV;
	}

	/* Configure interrupts */
	dev_cfg->irq_config();

	/* Initialize tx semaphores */
	k_sem_init(&dev_data->tx.sem, CONFIG_I2S_SEDI_TX_BLOCK_COUNT,
		   CONFIG_I2S_SEDI_TX_BLOCK_COUNT);

	/* Initialize rx semaphores */
	k_sem_init(&dev_data->rx.sem, 0, CONFIG_I2S_SEDI_RX_BLOCK_COUNT);

	/* Initialize tx/rx dma completion alert on stop/pause request. */
	k_sem_init(&dev_data->tx.dma_stop_alert,
		   0, I2S_CANCEL_PENDING_MAX);
	k_sem_init(&dev_data->rx.dma_stop_alert,
		   0, I2S_CANCEL_PENDING_MAX);
#if ZEPHYR_DMA_DRV
	static struct i2s_dma_ctx_t i2s_dma_ctx[MAX_I2S_INSTANCE];

	dev_data->dev_dma = device_get_binding(CONFIG_I2S_SEDI_DMA_NAME);
	if (!dev_data->dev_dma) {
		I2S_LOG_ERR("%s device not found", CONFIG_I2S_SEDI_DMA_NAME);
		return -ENODEV;
	}

	i2s_dma_ctx[dev_data->dev_instance].tx_chnl = dev_data->tx.dma_channel;
	i2s_dma_ctx[dev_data->dev_instance].rx_chnl = dev_data->rx.dma_channel;
	i2s_dma_ctx[dev_data->dev_instance].dev = dev;
#else
	strm_tx->dma_cfg.data = (uint8_t *)dev_data;
	strm_rx->dma_cfg.data = (uint8_t *)dev_data;
#endif

	/* Enable module's IRQ */
	irq_enable(dev_cfg->irq_id);

	LOG_INF("Device %s initialized", DEV_NAME(dev));
	return 0;
}

static const struct i2s_driver_api i2s_sedi_driver_api = {
	.configure = i2s_sedi_configure,
	.config_get = i2s_sedi_config_get,
	.write = i2s_sedi_write,
	.read = i2s_sedi_read,
	.trigger = i2s_sedi_trigger,
};

#if ZEPHYR_DMA_DRV
#define CREATE_I2S_INSTANCE(num)							 \
	DEVICE_DECLARE(i2s_##num);							 \
	struct queue_item								 \
		i2s##num##_tx_ring_buf[CONFIG_I2S_SEDI_TX_BLOCK_COUNT];			 \
	struct queue_item								 \
		i2s##num##_rx_ring_buf[CONFIG_I2S_SEDI_RX_BLOCK_COUNT];			 \
											 \
	static void i2s##num##_irq_config(void)						 \
	{										 \
		I2S_LOG_TRACE("%s\n", __func__);					 \
		IRQ_CONNECT(DT_INST_IRQN(num),						 \
			    DT_INST_IRQ(num, priority), i2s_sedi_isr,			 \
			    DEVICE_GET(i2s_##num), 0);					 \
	}										 \
											 \
	static const struct i2s_sedi_config i2s##num##_sedi_config = {			 \
		.irq_id = DT_INST_IRQN(num),						 \
		.irq_config = i2s##num##_irq_config,					 \
	};										 \
											 \
	static struct i2s_sedi_dev_data i2s##num##_sedi_data = {			 \
		.dev_instance = DT_INST_PROP(num, peripheral_id),			 \
		.i2s_mode = DT_INST_PROP(num, i2s_mode),				 \
		.tx =									 \
		{									 \
			.dma_channel = DT_INST_PROP(num, dma_tx_channel),		 \
			.dma_cfg =							 \
			{								 \
				.source_data_size = I2S_DATA_WIDTH_1B,			 \
				.dest_data_size = I2S_DATA_WIDTH_1B,			 \
				.source_burst_length =					 \
					I2S_DMA_SRC_BURST_LEN,				 \
				.dest_burst_length =					 \
					I2S_DMA_DST_BURST_LEN,				 \
				.dma_callback = dma_tx_callback,			 \
				.complete_callback_en = TRUE,				 \
				.error_callback_en = TRUE,				 \
				.block_count = I2S_DMA_BLOCK_CNT,			 \
				.channel_direction = MEMORY_TO_PERIPHERAL,		 \
				.reserved = DT_INST_PROP(num, dma_hwid),		 \
			},								 \
			.mem_block_queue.buf = i2s##num##_tx_ring_buf,			 \
			.mem_block_queue.len =						 \
				ARRAY_SIZE(i2s##num##_tx_ring_buf),			 \
			.stream_start = tx_stream_start,				 \
			.stream_disable = tx_stream_disable,				 \
			.queue_drop = tx_queue_drop,					 \
			.state = I2S_STATE_NOT_READY,					 \
			.hw_fifo = DT_INST_PROP(num, hw_fifo_addr),			 \
		},									 \
		.rx =									 \
		{									 \
			.dma_channel =							 \
				DT_INST_PROP(num, dma_rx_channel),			 \
			.dma_cfg =							 \
			{								 \
				.source_data_size = I2S_DATA_WIDTH_1B,			 \
				.dest_data_size = I2S_DATA_WIDTH_1B,			 \
				.source_burst_length =					 \
					I2S_DMA_SRC_BURST_LEN,				 \
				.dest_burst_length =					 \
					I2S_DMA_DST_BURST_LEN,				 \
				.dma_callback = dma_rx_callback,			 \
				.complete_callback_en = TRUE,				 \
				.error_callback_en = TRUE,				 \
				.block_count = I2S_DMA_BLOCK_CNT,			 \
				.channel_direction = PERIPHERAL_TO_MEMORY,		 \
				.reserved = DT_INST_PROP(num, dma_hwid),		 \
			},								 \
			.mem_block_queue.buf = i2s##num##_rx_ring_buf,			 \
			.mem_block_queue.len =						 \
				ARRAY_SIZE(i2s##num##_rx_ring_buf),			 \
			.stream_start = rx_stream_start,				 \
			.stream_disable = rx_stream_disable,				 \
			.queue_drop = rx_queue_drop,					 \
			.state = I2S_STATE_NOT_READY,					 \
			.hw_fifo = DT_INST_PROP(num, hw_fifo_addr),			 \
		},									 \
	};										 \
											 \
	DEVICE_DEFINE(i2s_##num, DT_INST_LABEL(num),					 \
		      &i2s_sedi_initialize, i2s_sedi_device_ctrl, &i2s##num##_sedi_data, \
		      &i2s##num##_sedi_config, POST_KERNEL,				 \
		      CONFIG_I2S_INIT_PRIORITY, &i2s_sedi_driver_api);
#else
#define CREATE_I2S_INSTANCE(num)					       \
	DEVICE_DECLARE(i2s_##num);					       \
	struct queue_item						       \
		i2s##num##_tx_ring_buf[CONFIG_I2S_SEDI_TX_BLOCK_COUNT];	       \
	struct queue_item						       \
		i2s##num##_rx_ring_buf[CONFIG_I2S_SEDI_RX_BLOCK_COUNT];	       \
									       \
	static void i2s##num##_irq_config(void)				       \
	{								       \
		I2S_LOG_TRACE("%s\n", __func__);			       \
		IRQ_CONNECT(DT_INST_IRQN(num),				       \
			    DT_INST_IRQ(num, priority), i2s_sedi_isr,	       \
			    DEVICE_GET(i2s_##num), 0);			       \
	}								       \
									       \
	static const struct i2s_sedi_config i2s##num##_sedi_config = {	       \
		.irq_id = DT_INST_IRQN(num),				       \
		.irq_config = i2s##num##_irq_config,			       \
	};								       \
									       \
	static struct i2s_sedi_dev_data i2s##num##_sedi_data = {	       \
		.dev_instance = DT_INST_PROP(num, peripheral_id),	       \
		.i2s_mode = DT_INST_PROP(num, i2s_mode),		       \
		.tx =							       \
		{							       \
			.dma_cfg =					       \
			{						       \
				.channel = DT_INST_PROP(num, dma_tx_channel),  \
				.source_data_size = I2S_DATA_WIDTH_1B,	       \
				.dest_data_size = I2S_DATA_WIDTH_1B,	       \
				.source_burst_length =			       \
					I2S_DMA_SRC_BURST_LEN,		       \
				.dest_burst_length =			       \
					I2S_DMA_DST_BURST_LEN,		       \
				.dma_callback = dma_tx_callback,	       \
				.complete_callback_en = TRUE,		       \
				.error_callback_en = TRUE,		       \
				.block_count = I2S_DMA_BLOCK_CNT,	       \
				.channel_direction = DMA_MEMORY_TO_PERIPHERAL, \
				.dev_hwid_dir = DMA_HS_PER_TX,		       \
				.dev_hwid = DT_INST_PROP(num, dma_hwid),       \
			},						       \
			.mem_block_queue.buf = i2s##num##_tx_ring_buf,	       \
			.mem_block_queue.len =				       \
				ARRAY_SIZE(i2s##num##_tx_ring_buf),	       \
			.stream_start = tx_stream_start,		       \
			.stream_disable = tx_stream_disable,		       \
			.queue_drop = tx_queue_drop,			       \
			.state = I2S_STATE_NOT_READY,			       \
			.hw_fifo = DT_INST_PROP(num, hw_fifo_addr),	       \
		},							       \
		.rx =							       \
		{							       \
			.dma_cfg =					       \
			{						       \
				.channel =				       \
					DT_INST_PROP(num, dma_rx_channel),     \
				.source_data_size = I2S_DATA_WIDTH_1B,	       \
				.dest_data_size = I2S_DATA_WIDTH_1B,	       \
				.source_burst_length =			       \
					I2S_DMA_SRC_BURST_LEN,		       \
				.dest_burst_length =			       \
					I2S_DMA_DST_BURST_LEN,		       \
				.dma_callback = dma_rx_callback,	       \
				.complete_callback_en = TRUE,		       \
				.error_callback_en = TRUE,		       \
				.block_count = I2S_DMA_BLOCK_CNT,	       \
				.channel_direction = DMA_PERIPHERAL_TO_MEMORY, \
				.dev_hwid_dir = DMA_HS_PER_RX,		       \
				.dev_hwid = DT_INST_PROP(num, dma_hwid),       \
			},						       \
			.mem_block_queue.buf = i2s##num##_rx_ring_buf,	       \
			.mem_block_queue.len =				       \
				ARRAY_SIZE(i2s##num##_rx_ring_buf),	       \
			.stream_start = rx_stream_start,		       \
			.stream_disable = rx_stream_disable,		       \
			.queue_drop = rx_queue_drop,			       \
			.state = I2S_STATE_NOT_READY,			       \
			.hw_fifo = DT_INST_PROP(num, hw_fifo_addr),	       \
		},							       \
	};								       \
									       \
	DEVICE_DEFINE(i2s_##num, DT_INST_LABEL(num),			       \
		      &i2s_sedi_initialize, NULL, &i2s##num##_sedi_data,       \
		      &i2s##num##_sedi_config, POST_KERNEL,		       \
		      CONFIG_I2S_INIT_PRIORITY, &i2s_sedi_driver_api);
#endif

DT_INST_FOREACH_STATUS_OKAY(CREATE_I2S_INSTANCE)
