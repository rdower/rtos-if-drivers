/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <sys/util.h>
#include <sys/printk.h>

#include <device.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>

#include <audio/codec.h>
#include "alc_5660I.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(alc_5660I);

#define USE_MCLK_GEN_CODEC_CLK         (0)
#define PRINT_ALC_CODEC_DFLT_REG_VAL   (0)

#define CODEC_OUTPUT_VOLUME_MAX         0 /* Limited to 0dB, actual:+12dB */
#define CODEC_OUTPUT_VOLUME_MIN         (-46.5)
#define CODEC_OUTPUT_VOLUME_STP         (1.5)
#define CODEC_OUTPUT_VOLUME_RANGE       (0x27)

#define CODEC_RESET_PIN_ASSERT          0
#define CODEC_RESET_PIN_DEASSERT        1

#define SHIFT_8_BITS    (8)

#define WRITE_REG_ADDR  1
#define READ_DATA_REG   2
#define WR_REG_DATA_VAL 3

#define CODEC_INIT_TIME 600
#define NUM_CODEC_REGS  256
#define MIN_M_DIV_VAL   2
#define OFFSET_OF_2     2
#define CENT            100
#define VOL_POINT_5     5
#define DFLT_ADC_DIGIVOL 0x2F

struct codec_driver_config {
	const struct device   *i2c_device;
	const char      *i2c_dev_name;
	uint8_t i2c_address;
	const struct device   *gpio_device;
	const char      *gpio_dev_name;
	uint32_t gpio_pin;
	int gpio_flags;
};

struct codec_driver_data {
	alc_reg_write_read_t reg_wr_rd;
};

static struct codec_driver_config codec_device_config = {
	.i2c_device = NULL,
	.i2c_dev_name = CONFIG_ALC_CODEC_I2C_MASTER_NAME,
	.i2c_address = (uint8_t)CONFIG_ALC_CODEC_ADDR,
	.gpio_device = NULL,
	.gpio_dev_name = CONFIG_ALC_CODEC_GPIO_NAME,
	.gpio_pin = CONFIG_GPIO_SEDI_0_PINS,
	.gpio_flags = 0,
};

static struct codec_driver_data codec_device_data;

#define DEV_CFG(dev) \
	((struct codec_driver_config *const)(dev)->config)
#define DEV_DATA(dev) \
	((struct codec_driver_data *const)(dev)->data)

static int codec_write_reg(const struct device *dev,
			   uint8_t reg_addr,
			   uint16_t val,
			   uint8_t num_bytes);
static int codec_read_reg(const struct device *dev, uint8_t reg, uint16_t *val);
static int codec_write_priv_reg(const struct device *dev, uint16_t reg_addr,
				uint16_t reg_val);
static int codec_soft_reset(const struct device *dev);
static int codec_configure_dai(const struct device *dev, audio_dai_cfg_t *cfg);
static int codec_configure_clocks(const struct device *dev,
				  struct audio_codec_cfg *cfg);
static int codec_configure_output(const struct device *dev);
static int codec_configure_input(const struct device *dev);
static int codec_set_output_volume(const struct device *dev, int vol);
static int codec_mute_output(const struct device *dev);
static void codec_start_output(const struct device *dev);

static int codec_initialize(const struct device *dev)
{
	struct codec_driver_config *const dev_cfg = DEV_CFG(dev);

	/* bind I2C */
	dev_cfg->i2c_device = device_get_binding(dev_cfg->i2c_dev_name);

	if (dev_cfg->i2c_device == NULL) {
		LOG_ERR("I2C device binding error");
		return -ENODEV;
	}

	/* bind GPIO */
	dev_cfg->gpio_device = device_get_binding(dev_cfg->gpio_dev_name);

	if (dev_cfg->gpio_device == NULL) {
		LOG_ERR("GPIO device binding error");
		return -ENODEV;
	}

	return 0;
}

static int codec_configure(const struct device *dev,
			   struct audio_codec_cfg *cfg)
{
	int ret = 0;
	pwr_mgmt_ctrl_1_t pwr_mgmt_ctrl1;
	pwr_mgmt_ctrl_2_t pwr_mgmt_ctrl2;
	pwr_mgmt_ctrl_3_t pwr_mgmt_ctrl3;
	pwr_mgmt_ctrl_4_t pwr_mgmt_ctrl4;
	pwr_mgmt_ctrl_5_t pwr_mgmt_ctrl5;
	pwr_mgmt_ctrl_6_t pwr_mgmt_ctrl6;
	sftvol_zcd_ctrl1_t sftvol_zcd1;
	sftvol_zcd_ctrl2_t sftvol_zcd2;
	mic_bias_ctrl_t mic_bias_ctrl;
	lout_amp_ctrl1_t lout_ctrl1;

	if (cfg->dai_type != AUDIO_DAI_TYPE_I2S) {
		LOG_ERR("dai_type must be AUDIO_DAI_TYPE_I2S");
		return -EINVAL;
	}

#if CONFIG_GPIO_CODEC_DET_PLUGIN
	struct codec_driver_config *const dev_cfg = DEV_CFG(dev);

	/* configure reset GPIO */
	ret = gpio_pin_configure(dev_cfg->gpio_device, dev_cfg->gpio_pin,
				 dev_cfg->gpio_flags);
	if (ret != 0) {
		goto exit;
	}
	/* de-assert reset */
	ret = gpio_pin_set(dev_cfg->gpio_device, dev_cfg->gpio_pin,
			   CODEC_RESET_PIN_DEASSERT);
	if (ret != 0) {
		goto exit;
	}

#endif
	ret = codec_soft_reset(dev);
	if (ret != 0) {
		goto exit;
	}
	/* Power Management registers */
	ret = codec_read_reg(dev, PWR_MGMT_CTRL_1, &pwr_mgmt_ctrl1.bits);
	if (ret != 0) {
		goto exit;
	}
	pwr_mgmt_ctrl1.en_i2s1 = SET_BIT;
	pwr_mgmt_ctrl1.pow_dac_l = SET_BIT;
	pwr_mgmt_ctrl1.pow_dac_r = SET_BIT;
	pwr_mgmt_ctrl1.pow_adc_l = SET_BIT;
	pwr_mgmt_ctrl1.pow_adc_r = SET_BIT;
	pwr_mgmt_ctrl1.pow_clsd = SET_BIT;
	ret = codec_write_reg(dev, PWR_MGMT_CTRL_1, pwr_mgmt_ctrl1.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	ret = codec_read_reg(dev, PWR_MGMT_CTRL_2, &pwr_mgmt_ctrl2.bits);
	if (ret != 0) {
		goto exit;
	}
	pwr_mgmt_ctrl2.pow_adc_stereo1_fltr = SET_BIT;
	pwr_mgmt_ctrl2.pow_dac_stereo1_fltr = SET_BIT;
	ret = codec_write_reg(dev, PWR_MGMT_CTRL_2, pwr_mgmt_ctrl2.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	ret = codec_read_reg(dev, PWR_MGMT_CTRL_3, &pwr_mgmt_ctrl3.bits);
	if (ret != 0) {
		goto exit;
	}
	pwr_mgmt_ctrl3.pow_vref1 = SET_BIT;
	pwr_mgmt_ctrl3.en_fastb1 = SET_BIT;
	pwr_mgmt_ctrl3.pow_main_bias = SET_BIT;
	pwr_mgmt_ctrl3.pow_bg_bias = SET_BIT;
	pwr_mgmt_ctrl3.en_l_hp = SET_BIT;
	pwr_mgmt_ctrl3.en_r_hp = SET_BIT;
	pwr_mgmt_ctrl3.en_amp_hp = SET_BIT;
	pwr_mgmt_ctrl3.pow_vref2 = SET_BIT;
	pwr_mgmt_ctrl3.en_fastb2 = SET_BIT;
	pwr_mgmt_ctrl3.pow_ldo2 = SET_BIT;
	ret = codec_write_reg(dev, PWR_MGMT_CTRL_3, pwr_mgmt_ctrl3.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	ret = codec_read_reg(dev, PWR_MGMT_CTRL_4, &pwr_mgmt_ctrl4.bits);
	if (ret != 0) {
		goto exit;
	}
	pwr_mgmt_ctrl4.pow_bst2 = SET_BIT;
	pwr_mgmt_ctrl4.pow_micbias1 = SET_BIT;
	pwr_mgmt_ctrl4.pow_micbias2 = SET_BIT;
	pwr_mgmt_ctrl4.pow_pll = SET_BIT;
	ret = codec_write_reg(dev, PWR_MGMT_CTRL_4, pwr_mgmt_ctrl4.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	ret = codec_read_reg(dev, PWR_MGMT_CTRL_5, &pwr_mgmt_ctrl5.bits);
	if (ret != 0) {
		goto exit;
	}
	pwr_mgmt_ctrl5.pow_outmixl = SET_BIT;
	pwr_mgmt_ctrl5.pow_outmixr = SET_BIT;
	pwr_mgmt_ctrl5.pow_spkmix = SET_BIT;
	pwr_mgmt_ctrl5.pow_recmixl = SET_BIT;
	pwr_mgmt_ctrl5.pow_recmixr = SET_BIT;
	ret = codec_write_reg(dev, PWR_MGMT_CTRL_5, pwr_mgmt_ctrl5.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	ret = codec_read_reg(dev, PWR_MGMT_CTRL_6, &pwr_mgmt_ctrl6.bits);
	if (ret != 0) {
		goto exit;
	}
	pwr_mgmt_ctrl6.pow_spkvol = SET_BIT;
	pwr_mgmt_ctrl6.pow_loutvoll = SET_BIT;
	pwr_mgmt_ctrl6.pow_loutvolr = SET_BIT;
	ret = codec_write_reg(dev, PWR_MGMT_CTRL_6, pwr_mgmt_ctrl6.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	ret = codec_read_reg(dev, SFTVOL_ZCD_CTRL_1, &sftvol_zcd1.bits);
	if (ret != 0) {
		goto exit;
	}
	sftvol_zcd1.en_softvol = SET_BIT;
	sftvol_zcd1.en_spo_svol = SET_BIT;
	sftvol_zcd1.en_o_svol = SET_BIT;
	sftvol_zcd1.en_zcd_digi = SET_BIT;
	sftvol_zcd1.pow_zcd = SET_BIT;
	ret = codec_write_reg(dev, SFTVOL_ZCD_CTRL_1, sftvol_zcd1.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}


	ret = codec_read_reg(dev, SFTVOL_ZCD_CTRL_2, &sftvol_zcd2.bits);
	if (ret != 0) {
		goto exit;
	}
	sftvol_zcd2.en_zcd_outmixr = SET_BIT;
	sftvol_zcd2.en_zcd_outmixl = SET_BIT;
	ret = codec_write_reg(dev, SFTVOL_ZCD_CTRL_2, sftvol_zcd2.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	ret = codec_read_reg(dev, MICBIAS_CTRL, &mic_bias_ctrl.bits);
	if (ret != 0) {
		goto exit;
	}
	mic_bias_ctrl.pow_int_clk = SET_BIT;
	ret = codec_write_reg(dev, MICBIAS_CTRL, mic_bias_ctrl.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	ret = codec_read_reg(dev, LOUT_AMP_CTRL_1, &lout_ctrl1.bits);
	if (ret != 0) {
		goto exit;
	}
	lout_ctrl1.pow_lout = SET_BIT;
	lout_ctrl1.en_out_lout = SET_BIT;
	ret = codec_write_reg(dev, LOUT_AMP_CTRL_1, lout_ctrl1.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	k_sleep(SYS_TIMEOUT_MS(CODEC_INIT_TIME));

	ret = codec_configure_clocks(dev, cfg);
	if (ret != 0) {
		goto exit;
	}
	ret = codec_configure_dai(dev, &cfg->dai_cfg);
	if (ret != 0) {
		goto exit;
	}
	ret = codec_configure_input(dev);
	if (ret != 0) {
		goto exit;
	}
	ret = codec_configure_output(dev);
	if (ret != 0) {
		goto exit;
	}
	codec_start_output(dev);

	ret = codec_write_reg(dev, STEREO_ADC_DAC_MIXER_CTRL_REG,
			      STEREO_ADC_DAC_MIXER_CTRL_REG_VAL,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}
	ret = codec_write_reg(dev, CLASS_D_AMP_GAIN_CTRL,
			      CLASS_D_AMP_CTRL_VAL, WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	ret = codec_write_reg(dev, SPKR_OUT_MIXER_CTRL_REG,
			      SPKR_OUT_MIXER_CTRL_REG_VAL, WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}
exit:
	return ret;
}

static void codec_start_output(const struct device *dev)
{
	pwr_mgmt_ctrl_1_t pwr_mgmt_1;
	int ret = 0;

	/* Enable i2s, DAC and CLASS-D amplifier
	 * in Power Management Register 0x61
	 */
	ret = codec_read_reg(dev, PWR_MGMT_CTRL_1, &pwr_mgmt_1.bits);
	if (ret != 0) {
		LOG_ERR("Reading from codec has failed %s %d\n", __func__,
			__LINE__);
		goto exit;
	}
	pwr_mgmt_1.en_i2s1 = PWR_ON;
	pwr_mgmt_1.pow_dac_l = PWR_ON;
	pwr_mgmt_1.pow_dac_r = PWR_ON;
	pwr_mgmt_1.pow_clsd = PWR_ON;
	ret = codec_write_reg(dev, PWR_MGMT_CTRL_1, pwr_mgmt_1.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		LOG_ERR("Writing to codec has failed %s %d\n", __func__,
			__LINE__);
		goto exit;
	}
exit:
	return;
}

static void codec_stop_output(const struct device *dev)
{
	pwr_mgmt_ctrl_1_t pwr_mgmt_1;
	int ret = 0;

	/* Speaker Output control Register 0x01
	 * Speaker volume is maintained at 0dB
	 */
	ret = codec_mute_output(dev);
	if (ret != 0) {
		LOG_ERR("Muting codec output has failed %s %d\n", __func__,
			__LINE__);
		goto exit;
	}

	/* Disable i2s, DAC and CLASS-D amplifier
	 * in Power Management Register 0x61
	 */
	ret = codec_read_reg(dev, PWR_MGMT_CTRL_1, &pwr_mgmt_1.bits);
	if (ret != 0) {
		LOG_ERR("Reading from codec has failed %s %d\n", __func__,
			__LINE__);
		goto exit;
	}
	pwr_mgmt_1.en_i2s1 = PWR_DOWN;
	pwr_mgmt_1.pow_dac_l = PWR_DOWN;
	pwr_mgmt_1.pow_dac_r = PWR_DOWN;
	pwr_mgmt_1.pow_clsd = PWR_DOWN;
	ret = codec_write_reg(dev, PWR_MGMT_CTRL_1, pwr_mgmt_1.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		LOG_ERR("Writing to codec has failed %s %d\n", __func__,
			__LINE__);
		goto exit;
	}

exit:
	return;

}

static int codec_mute_output(const struct device *dev)
{
	speaker_op_ctrl_t spkr_out_ctrl;
	int ret = 0;

	/* Speaker Output control Register 0x01
	 * Speaker volume is maintained at 0dB
	 */
	ret = codec_read_reg(dev, SPKR_OUT_CTRL_REG, &spkr_out_ctrl.bits);
	if (ret != 0) {
		goto exit;
	}
	spkr_out_ctrl.mu_spkr_vol = MUTE;
	ret = codec_write_reg(dev, SPKR_OUT_CTRL_REG, spkr_out_ctrl.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}
exit:
	return ret;

}

static int codec_unmute_output(const struct device *dev)
{
	speaker_op_ctrl_t spkr_out_ctrl;
	int ret = 0;

	/* Speaker Output control Register 0x01
	 * Speaker volume is maintained at 0dB
	 */
	ret = codec_read_reg(dev, SPKR_OUT_CTRL_REG, &spkr_out_ctrl.bits);
	if (ret != 0) {
		goto exit;
	}
	spkr_out_ctrl.mu_spkr_vol = UNMUTE;
	ret = codec_write_reg(dev, SPKR_OUT_CTRL_REG, spkr_out_ctrl.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}
exit:
	return ret;
}

static int codec_set_property(const struct device *dev,
			      audio_property_t property,
			      audio_channel_t channel,
			      audio_property_value_t val)
{
	/* individual channel control not currently supported */
	if (channel != AUDIO_CHANNEL_ALL) {
		LOG_ERR("channel %u invalid. must be AUDIO_CHANNEL_ALL",
			channel);
		return -EINVAL;
	}

	switch (property) {
	case AUDIO_PROPERTY_OUTPUT_VOLUME:
		return codec_set_output_volume(dev, val.vol);

	case AUDIO_PROPERTY_OUTPUT_MUTE:
		if (val.mute) {
			return codec_mute_output(dev);
		} else {
			return codec_unmute_output(dev);
		}
		return 0;

	default:
		break;
	}

	return -EINVAL;
}

static int codec_apply_properties(const struct device *dev)
{
	/* nothing to do because there is nothing cached */
	return 0;
}

static int codec_write_reg(const struct device *dev,
			   uint8_t reg_addr,
			   uint16_t val,
			   uint8_t num_bytes)
{
	struct codec_driver_config *const dev_cfg = DEV_CFG(dev);
	alc_reg_write_read_t alc_reg;

	alc_reg.reg_addr = reg_addr;
	if (num_bytes > WRITE_REG_ADDR) {
		alc_reg.reg_high_byte = (uint8_t)(val >> SHIFT_8_BITS);
		alc_reg.reg_low_byte =  (uint8_t)val;
	}
	return i2c_write(dev_cfg->i2c_device, (uint8_t *)&alc_reg, num_bytes,
			 dev_cfg->i2c_address);
}

static int codec_read_reg(const struct device *dev,
			  uint8_t reg_addr,
			  uint16_t *val)
{
	uint16_t reg_value = 0;
	int ret = 0;
	struct codec_driver_config *const dev_cfg = DEV_CFG(dev);

	/* Read data from the ALC-5660I codec register
	 * Swap register value, because high byte transimitted
	 * first by the codec.
	 * Copy back the data to the input data pointer
	 */
	ret = i2c_write_read(dev_cfg->i2c_device, dev_cfg->i2c_address,
			     &reg_addr, WRITE_REG_ADDR, &reg_value,
			     READ_DATA_REG);
	if (ret != 0) {
		goto exit;
	}
	*val =  (uint8_t)(reg_value >> SHIFT_8_BITS);
	*(val + 1) = (uint8_t)reg_value;
	*val = *val | (*(val + 1) << SHIFT_8_BITS);
exit:
	return ret;
}


static int codec_write_priv_reg(const struct device *dev, uint16_t reg_addr,
				uint16_t reg_val)
{
	int ret = 0;

	ret = codec_write_reg(dev, PRIV_REG_IDX, reg_addr, WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}
	ret = codec_write_reg(dev, PRIV_REG_DATA, reg_val, WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}
exit:
	return ret;
}

#if PRINT_ALC_CODEC_DFLT_REG_VAL
static void codec_read_all_regs(const struct device *dev)
{
	uint16_t codec_reg_val;
	uint8_t reg_addr;


	/* Read All ALC_5660I codec registers */
	for (reg_addr = 0; reg_addr < NUM_CODEC_REGS; reg_addr++) {
		codec_read_reg(dev, reg_addr, &codec_reg_val);
		LOG_DBG("\n Reg No: 0x%02X, Reg Value:0x%04X", reg_addr,
			codec_reg_val);
	}
}
#endif

static int codec_soft_reset(const struct device *dev)
{
	/* soft reset the DAC */
	return codec_write_reg(dev, SOFT_RESET_CTRL_REG, 0, WR_REG_DATA_VAL);

}

static int codec_configure_dai(const struct device *dev, audio_dai_cfg_t *cfg)
{
	i2s1_digi_intf_ctrl_t intf_ctrl = { 0, };
	int ret = 0;

	/* configure I2S interface */
	if ((cfg->i2s.options == I2S_OPT_FRAME_CLK_MASTER) &&
	    (cfg->i2s.options == I2S_OPT_BIT_CLK_MASTER)) {
		intf_ctrl.sel_i2s1_ms = MASTER_MODE;
	} else {
		intf_ctrl.sel_i2s1_ms = SLAVE_MODE;
	}

	/* Configure BCLK polarity to zero */
	intf_ctrl.inv_i2s1_bclk = NORMAL;

	switch (cfg->i2s.word_size) {
	case AUDIO_PCM_WIDTH_16_BITS:
		intf_ctrl.sel_i2s1_len = BITS_16;
		break;
	case AUDIO_PCM_WIDTH_20_BITS:
		intf_ctrl.sel_i2s1_len = BITS_20;
		break;
	case AUDIO_PCM_WIDTH_24_BITS:
		intf_ctrl.sel_i2s1_len = BITS_24;
		break;
	default:
		LOG_ERR("Unsupported PCM sample bit width %u",
			cfg->i2s.word_size);
		ret = -EINVAL;
		goto exit;
	}

	switch (cfg->i2s.format) {
	case I2S_FMT_DATA_FORMAT_I2S:
		intf_ctrl.sel_i2s1_frmt = I2S_STD_FRMT;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
		intf_ctrl.sel_i2s1_frmt = PCM_MODE_A;
		break;
	case  I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		intf_ctrl.sel_i2s1_frmt = LEFT_JUSTIFIED;
		break;
	case  I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
	case I2S_FMT_DATA_FORMAT_PCM_LONG:
	default:
		LOG_ERR("Unsupported I2S format %u", cfg->i2s.format);
		ret = -EINVAL;
		goto exit;
	}

	ret = codec_write_reg(dev, I2S1_DIGI_IF_CTRL_REG, intf_ctrl.bits,
			      WR_REG_DATA_VAL);

exit:
	return ret;
}

static int codec_configure_clocks(const struct device *dev,
				  struct audio_codec_cfg *cfg)
{
#if USE_MCLK_GEN_CODEC_CLK
	int dac_clk, mod_clk;
	uint32_t mclk, sysclk, ndiv, mdiv, nm_sysclk, exit_loop = 0;
	int sysclk_err;
	uint16_t alc_reg_val;
	pll_ctrl_1_t pll_ctrl_1;
	pll_ctrl_2_t pll_ctrl_2;
#endif
	struct i2s_config *i2s;
	glbl_clk_ctrl_t glbl_clk_ctrl;
	gen_ctrl_1_t gen_ctrl_1;
	int ret = 0;

	i2s = &cfg->dai_cfg.i2s;
	LOG_DBG("MCLK %u Hz PCM Rate: %u Hz", cfg->mclk_freq,
		i2s->frame_clk_freq);
#if USE_MCLK_GEN_CODEC_CLK   /* Enable when MCLK is provided to Codec */
	sysclk = ALC_SYS_CLK_MULTI * ALC_DIV_F1 * i2s->frame_clk_freq;

	for (ndiv = 0; ndiv < PLL_MAX_N_VALUE; ndiv++) {
		for (mdiv = MIN_M_DIV_VAL; mdiv <= PLL_MAX_M_VALUE; mdiv++) {
			nm_sysclk = (cfg->mclk_freq * (ndiv + OFFSET_OF_2)) /
				    ((mdiv + OFFSET_OF_2) *
				     (PLL_DEFAULT_K_VALUE + OFFSET_OF_2));
			sysclk_err = ((sysclk - nm_sysclk) * CENT) / sysclk;

			if (!sysclk_err) {
				exit_loop = 1;
				break;
			}
		}
		if (exit_loop) {
			break;
		}
	}

	mdiv -= MIN_M_DIV_VAL; /* 0: Divide by 2 0xF: divide by 17 */

	LOG_DBG("ndiv  %u mdiv : %u\n", ndiv, mdiv);

	ret = codec_read_reg(dev, PLL_CTRL1_REG, &pll_ctrl_1.bits);
	if (ret != 0) {
		goto exit;
	}
	pll_ctrl_1.pll_n_code = ndiv;
	pll_ctrl_1.pll_k_code = PLL_DEFAULT_K_VALUE;
	ret = codec_write_reg(dev, PLL_CTRL1_REG, pll_ctrl_1.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	ret = codec_read_reg(dev, PLL_CTRL2_REG, &pll_ctrl_2.bits);
	if (ret != 0) {
		goto exit;
	}
	pll_ctrl_2.pll_m_code = mdiv;

	if (mdiv == 0) {
		pll_ctrl_2.pll_m_bypass = PLL_M_BYPASS;         /* bypass */
	} else {
		pll_ctrl_2.pll_m_bypass = PLL_M_NO_BYPASS;      /* No bypass */
	}
	ret = codec_write_reg(dev, PLL_CTRL2_REG, pll_ctrl_2.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	ret = codec_read_reg(dev, GLOBAL_CLOCK_CTRL_REG, &glbl_clk_ctrl.bits);
	if (ret != 0) {
		goto exit;
	}
	glbl_clk_ctrl.sel_sysclk_1 = SYS_CLK_SRC_PLL;
	ret = codec_write_reg(dev, GLOBAL_CLOCK_CTRL_REG, glbl_clk_ctrl.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	ret = codec_read_reg(dev, ADC_DAC_CLK_CTRL_1, &adc_dac_clk_ctrl.bits);
	if (ret != 0) {
		goto exit;
	}
	adc_dac_clk_ctrl.sel_i2s_pre_div1 = I2S_CLK_PRE_DIV_BY_2;
	ret = codec_write_reg(dev, ADC_DAC_CLK_CTRL_1, adc_dac_clk_ctrl.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}
#else   /* For validation @ sample frequency 16KHz */
	ret = codec_read_reg(dev, GEN_CTRL_1, &gen_ctrl_1.bits);
	if (ret != 0) {
		goto exit;
	}
	gen_ctrl_1.digi_gate_ctrl = ENABLE_MCLK_IP_CLK;
	ret = codec_write_reg(dev, GEN_CTRL_1, gen_ctrl_1.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	ret = codec_read_reg(dev, GLOBAL_CLOCK_CTRL_REG, &glbl_clk_ctrl.bits);
	if (ret != 0) {
		goto exit;
	}
	glbl_clk_ctrl.sel_pll_sour = PLL_CLK_SRC_INTER_CLK;
	glbl_clk_ctrl.sel_sysclk_1 = SYS_CLK_SRC_PLL;
	ret = codec_write_reg(dev, GLOBAL_CLOCK_CTRL_REG, glbl_clk_ctrl.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	ret = codec_write_reg(dev, PLL_CTRL1_REG, PLL_N_K_VALUE_16KHz,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	ret = codec_write_reg(dev, PLL_CTRL2_REG, PLL_M_VALUE_16KHz,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}
#endif
exit:
	return ret;
}

static int codec_configure_output(const struct device *dev)
{
	stereo_dac1_mixer_ctrl_t dac_mixer_ctrl;
	outmixl_ctrl_3_t outmixl_ctrl3;
	outmixr_ctrl_3_t outmixr_ctrl3;
	lineoutput_ctrl_t lineout_ctrl;
	lineout_mix_ctrl_t lineout_mix_ctrl;
	int ret = 0;

	/* Note:
	 * Digital Interface Data control Register 0x2F
	 * MONO MODE: I2S copy LEFT channel data to right channel
	 * also and vice versa. Hence normal mode of digital interface
	 * is enough(Default configuration).Hence no register update
	 *
	 * DAC L/R Digital volume control Register 0x19
	 * Default configuration(0xAF - 0dB) is maintained. Hence no register
	 * update
	 *
	 * Stereo ADC to DAC Digital Mixer control Register 0x29
	 * Default configuration(unmute IF1 to DAC) connects I2S Tx pin data to
	 * DAC. Hence no register update
	 */

	/* Stereo DAC Digital Mixer control Register 0x2A */
	/* Note: Gain is set to -6dB */
	ret = codec_read_reg(dev, STEREO_DAC_DIGI_MIXER_CTRL_REG,
			     &dac_mixer_ctrl.bits);
	if (ret != 0) {
		goto exit;
	}
	dac_mixer_ctrl.mu_stereo_dacl1_mixl = UNMUTE;
	dac_mixer_ctrl.gain_dacl1_stereo_r = SET_BIT;
	dac_mixer_ctrl.mu_stereo_dacr1_mixr = UNMUTE;
	dac_mixer_ctrl.gain_dacr1_stereo_l = SET_BIT;
	ret = codec_write_reg(dev, STEREO_DAC_DIGI_MIXER_CTRL_REG,
			      dac_mixer_ctrl.bits, WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	/* OutMixer Left control-3  Register 0x4F */
	ret = codec_read_reg(dev, OUT_MIXL_CTRL_3, &outmixl_ctrl3.bits);
	if (ret != 0) {
		goto exit;
	}
	outmixl_ctrl3.mu_dacl_outmixl = UNMUTE;
	outmixl_ctrl3.mu_bst2_outmixl = UNMUTE;
	outmixl_ctrl3.mu_recmixl_outmixl = UNMUTE;
	ret = codec_write_reg(dev, OUT_MIXL_CTRL_3, outmixl_ctrl3.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	/* OutMixer Right control-3  Register 0x52 */
	ret = codec_read_reg(dev, OUT_MIXR_CTRL_3, &outmixr_ctrl3.bits);
	if (ret != 0) {
		goto exit;
	}
	outmixr_ctrl3.mu_dacr_outmixr = UNMUTE;
	outmixr_ctrl3.mu_bst2_outmixr = UNMUTE;
	outmixr_ctrl3.mu_recmixr_outmixr = UNMUTE;
	ret = codec_write_reg(dev, OUT_MIXR_CTRL_3, outmixr_ctrl3.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	/* Line output control register 0x02 */
	/* Note: Volume is maintained at 0dB */
	ret = codec_read_reg(dev, LINE_OUT_CTRL_REG, &lineout_ctrl.bits);
	if (ret != 0) {
		goto exit;
	}
	lineout_ctrl.mu_loutvoll_in = UNMUTE;
	lineout_ctrl.mu_loutvolr_in = UNMUTE;
	ret = codec_write_reg(dev, LINE_OUT_CTRL_REG, lineout_ctrl.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	/* LOUT	Mix control register 0x045 */
	/* Note: Gain is maintained at 0dB */
	ret = codec_read_reg(dev, LINE_OUT_MIX_CTRL, &lineout_mix_ctrl.bits);
	if (ret != 0) {
		goto exit;
	}
	lineout_mix_ctrl.mu_loutvol_loutmix = UNMUTE;
	ret = codec_write_reg(dev, LINE_OUT_MIX_CTRL, lineout_mix_ctrl.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	/* Line output control register 0x02 */
	/* Volume is maintained at 0dB */
	ret = codec_read_reg(dev, LINE_OUT_CTRL_REG, &lineout_ctrl.bits);
	if (ret != 0) {
		goto exit;
	}
	lineout_ctrl.mu_lout_r = UNMUTE;
	lineout_ctrl.mu_lout_l = UNMUTE;
	ret = codec_write_reg(dev, LINE_OUT_CTRL_REG, lineout_ctrl.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}
exit:
	return ret;
}

static int codec_configure_input(const struct device *dev)
{
	recmixl_ctrl_2_t recmixl_ctrl2;
	recmixr_ctrl_2_t recmixr_ctrl2;
	stereo1_adc_digi_mixer_ctrl_t stereo_adc_digi_mixer;
	int ret = 0;

	ret = codec_write_priv_reg(dev, PRIV_ADC_DAC_RST_CTRL,
				   ENABLE_ADC_DAC_CLK_GEN);
	if (ret != 0) {
		goto exit;
	}

	/* Record Mixer Left control  0x3C
	 * Connect BST1 to Record Mixer Left control.
	 */
	ret = codec_read_reg(dev, REC_MIXL_CTRL_2, &recmixl_ctrl2.bits);
	if (ret != 0) {
		goto exit;
	}
	recmixl_ctrl2.mu_bst2_recmixl = UNMUTE;
	ret = codec_write_reg(dev, REC_MIXL_CTRL_2, recmixl_ctrl2.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	/* Record Mixer Right control  0x3E
	 * Connect BST1 to Record Mixer Right control.
	 */
	ret = codec_read_reg(dev, REC_MIXR_CTRL_2, &recmixr_ctrl2.bits);
	if (ret != 0) {
		goto exit;
	}
	recmixr_ctrl2.mu_bst2_recmixr = UNMUTE;
	ret = codec_write_reg(dev, REC_MIXR_CTRL_2, recmixr_ctrl2.bits,
			      WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	/* Stereo ADC digital mixer control  0x27
	 * Connect ADC data to Digital mixer.
	 */
	ret = codec_read_reg(dev, STEREO_1_ADC_DIGI_MIXR_CTRL,
			     &stereo_adc_digi_mixer.bits);
	if (ret != 0) {
		goto exit;
	}
	stereo_adc_digi_mixer.mu_stereo_adcl1 = UNMUTE;
	stereo_adc_digi_mixer.mu_stereo_adcr1 = UNMUTE;
	ret = codec_write_reg(dev, STEREO_1_ADC_DIGI_MIXR_CTRL,
			      stereo_adc_digi_mixer.bits, WR_REG_DATA_VAL);
	if (ret != 0) {
		goto exit;
	}

	/* Note :
	 * ADC Digital Boost gain control 0x1E
	 * Digital boost gain is maintained at 0dB
	 * Hence no update to the register
	 *
	 * Stereo ADC volume  control  0x1C
	 * Note: Default Volume is at 0dB and volume channel is unmuted.
	 * Register is not modified
	 */

exit:
	return ret;
}

static int codec_set_output_volume(const struct device *dev, int vol)
{
	uint8_t vol_val;
	int ret = 0;

	if ((vol > CODEC_OUTPUT_VOLUME_MAX) ||
	    (vol < CODEC_OUTPUT_VOLUME_MIN)) {
		LOG_ERR("Invalid volume %d.%d dB",
			vol, ((uint32_t)vol & SET_BIT) ? VOL_POINT_5 : 0);
		ret = -EINVAL;
		goto exit;
	}

	/* calculate value from input volume  0x08:0dB 0x27:-46.5dB */
	vol_val = CODEC_OUTPUT_VOLUME_RANGE - ((CODEC_OUTPUT_VOLUME_MIN + vol)
					       / CODEC_OUTPUT_VOLUME_STP);

	ret = codec_write_reg(dev, SPKR_OUT_CTRL_REG, vol_val,
			      WR_REG_DATA_VAL);
exit:
	return ret;
}


static const struct audio_codec_api codec_driver_api = {
	.configure = codec_configure,
	.start_output = codec_start_output,
	.stop_output = codec_stop_output,
	.set_property = codec_set_property,
	.apply_properties = codec_apply_properties,
};

DEVICE_AND_API_INIT(alc5660I, "DT_ALC5660I_LABEL", codec_initialize,
		    &codec_device_data, &codec_device_config, POST_KERNEL,
		    CONFIG_AUDIO_CODEC_INIT_PRIORITY, &codec_driver_api);
