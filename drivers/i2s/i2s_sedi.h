/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief I2S bus (SSP) driver for Intel CAVS.
 *
 * Limitations:
 * - DMA is used in simple single block transfer mode (with linked list
 *   enabled) and "interrupt on full transfer completion" mode.
 */

#ifndef _I2S_CAVS_H_
#define _I2S_CAVS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define I2S_DMA_INSTANCE SEDI_DMA_0
#define ZEPHYR_DMA_DRV (0)

#define MAX_I2S_INSTANCE (SEDI_I2S_MAX)
#define I2S_0_INSTANCE (SEDI_I2S_NUM_0)
#define I2S_1_INSTANCE (SEDI_I2S_NUM_1)

#define I2S_EN_FULLDUPLEX_MODE (1)

#define BIT_WORD_LEN_12 (12)
#define BIT_WORD_LEN_16 (16)
#define BIT_WORD_LEN_24 (24)
#define BIT_WORD_LEN_32 (32)

#define I2S_DATA_WIDTH_1B (DMA_TRANS_WIDTH_8)
#define I2S_DATA_WIDTH_2B (DMA_TRANS_WIDTH_16)
#define I2S_DATA_WIDTH_4B (DMA_TRANS_WIDTH_32)

#define I2S_DMA_SRC_BURST_LEN (32)
#define I2S_DMA_DST_BURST_LEN (1)

#define I2S_DMA_BLOCK_CNT (1)

#define I2S_CANCEL_PENDING_MAX (1)
#ifdef __cplusplus
}
#endif

#endif /* _I2S_CAVS_H_ */
