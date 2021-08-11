/*
 * Copyright (c) 2021 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_AUDIO_ALC5660I_H_
#define  ZEPHYR_DRIVERS_AUDIO_ALC5660I_H_

#ifdef __cplusplus
extern "C" {
#endif


#define SET_BIT  1

/* Register addresses */
#define SOFT_RESET_CTRL_REG             0x00
#define SPKR_OUT_CTRL_REG               0x01
#define LINE_OUT_CTRL_REG               0x02
#define IN_1_2_INPUT_CTRL               0x0D
#define DAC_DIGI_VOLUME_REG             0x19
#define STEREO_1_ADC_DIGI_VOL_CTRL      0x1C
#define STEREO_1_ADC_DIGI_MIXR_CTRL     0x27
#define STEREO_ADC_DAC_MIXER_CTRL_REG   0x29
#define STEREO_DAC_DIGI_MIXER_CTRL_REG  0x2A
#define DIGI_IF_DATA_CTRL_REG           0x2F
#define REC_MIXL_CTRL_2                 0x3C
#define REC_MIXR_CTRL_1                 0x3D
#define REC_MIXR_CTRL_2                 0x3E
#define LINE_OUT_MIX_CTRL               0x45
#define SPKR_OUT_MIXER_CTRL_REG         0x48
#define OUT_MIXL_CTRL_3                 0x4F
#define OUT_MIXR_CTRL_3                 0x52
#define I2S1_DIGI_IF_CTRL_REG           0x70
#define ADC_DAC_CLK_CTRL_1              0x73
#define GLOBAL_CLOCK_CTRL_REG           0x80
#define PLL_CTRL1_REG                   0x81
#define PLL_CTRL2_REG                   0x82
#define MICBIAS_CTRL                    0x93

#define PWR_MGMT_CTRL_1 0x61
#define PWR_MGMT_CTRL_2 0x62
#define PWR_MGMT_CTRL_3 0x63
#define PWR_MGMT_CTRL_4 0x64
#define PWR_MGMT_CTRL_5 0x65
#define PWR_MGMT_CTRL_6 0x66

#define PRIV_REG_IDX    0x6A
#define PRIV_REG_DATA   0x6C

#define CLS_D_AMP_OUT_CTRL  0x8D
#define LOUT_AMP_CTRL_1     0x8E
#define DRC_AGC1_CTRL_2     0xB4
#define DRC_AGC1_CTRL_3     0xB5
#define SFTVOL_ZCD_CTRL_1   0xD9
#define SFTVOL_ZCD_CTRL_2   0xDA

#define GEN_CTRL_1          0xFA

/* Private Registers */
#define PRIV_ADC_DAC_RST_CTRL       0x3D
#define PRIV_DYN_WIND_FLTR_CTRL_1   0x50


/* PLL N/M divider values */
#define PLL_MAX_M_VALUE 17
#define PLL_MAX_N_VALUE 513
#define PLL_MAX_K_VALUE 33
#define PLL_DEFAULT_K_VALUE 0x02
#define PLL_N_K_VALUE_16KHz 0x1F00              /* N:33 K:2 */
#define PLL_M_VALUE_16KHz   0xB000              /* M:13 and Not bypassed */

#define ENABLE_DYN_WIND_FLTR 0x9AC5             /* Enable dynamic wind filter */
#define ENABLE_ADC_DAC_CLK_GEN 0x3600           /* Enable ADC/DAC clock generation */

#define ALC_PROC_CLK_FREQ_MAX   40000000        /* 40.00 MHz */
#define ALC_SYS_CLK_MULTI       256
#define ALC_DIV_F1              2

#define UNMUTE      (0)
#define MUTE        (1)

#define PWR_DOWN    (0)
#define PWR_ON      (1)

#define STEREO_ADC_DAC_MIXER_CTRL_REG_VAL 0x4040
#define CLASS_D_AMP_GAIN_CTRL 0x4A
#define CLASS_D_AMP_CTRL_VAL 0x0004
#define SPKR_OUT_MIXER_CTRL_REG_VAL 0xF800

typedef enum {
	NORMAL_NO_SWAP = 0,
	SWAP,
	LEFT_CHNL_CP_RIGHT_CHNL,
	RIGHT_CHNL_CP_LEFT_CHNL
} digi_intf_data_t;

typedef enum {
	OFF = 0,
	ULAW,
	ALAW,
	RESERVED_DATA_COMPRESS
} i2s_data_compress_t;

typedef enum {
	MASTER_MODE = 0,
	SLAVE_MODE
} i2s_intf_mode_t;

typedef enum {
	NORMAL = 0,
	INVERTED
} i2s_bclk_polarity_t;

typedef enum {
	BITS_16 = 0,
	BITS_20,
	BITS_24,
	BITS_08
} i2s_data_len_t;

typedef enum {
	I2S_STD_FRMT = 0,
	LEFT_JUSTIFIED,
	PCM_MODE_A,
	PCM_MODE_B
} i2s_data_format_t;

typedef enum {
	I2S_CLK_PRE_DIV_BY_1 = 0,
	I2S_CLK_PRE_DIV_BY_2,
	I2S_CLK_PRE_DIV_BY_3,
	I2S_CLK_PRE_DIV_BY_4,
	I2S_CLK_PRE_DIV_BY_6,
	I2S_CLK_PRE_DIV_BY_8,
	I2S_CLK_PRE_DIV_BY_12,
	I2S_CLK_PRE_DIV_BY_16
} i2s_pre_div_1_t;

typedef enum {
	STEREO_OSR_128FS = 0,
	STEREO_OSR_68FS,
	STEREO_OSR_32FS,
	STEREO_OSR_16FS
} stereo_over_sample_rate_t;

typedef enum {
	I2S_BCLK_MS_16_BITS = 0,
	I2S_BCLK_MS_32_BITS
} i2s_bclk_ms_t;

typedef enum {
	SYS_CLK_SRC_MCLK = 0,
	SYS_CLK_SRC_PLL,
	SYS_CLK_SRC_INTER_CLK,
	RESERVED_SYC_CLK_SRC
} sys_clk_src_t;

typedef enum {
	PLL_CLK_SRC_MCLK = 0,
	PLL_CLK_SRC_BCLK,
	PLL_CLK_SRC_INTER_CLK,
	RESERVED_PLL_CLK_SRC
} pll_clk_src_t;

typedef enum {
	PLL_PRE_DIVIDER_BY_1 = 0,
	PLL_PRE_DIVIDER_BY_2,
} pll_pre_divide_t;

typedef enum {
	SYS_CLK_DIV_FOR_DAC_FLTR_BY_1 = 0,
	SYS_CLK_DIV_FOR_DAC_FLTR_BY_2,
	SYS_CLK_DIV_FOR_DAC_FLTR_BY_3,
	SYS_CLK_DIV_FOR_DAC_FLTR_BY_4,
	SYS_CLK_DIV_FOR_DAC_FLTR_BY_6,
	SYS_CLK_DIV_FOR_DAC_FLTR_BY_8,
	SYS_CLK_DIV_FOR_DAC_FLTR_BY_12,
	SYS_CLK_DIV_FOR_DAC_FLTR_BY_16
} syclk_div_for_dac_fltr_t;

typedef enum {
	PLL_M_NO_BYPASS = 0,
	PLL_M_BYPASS
} pll_m_divide_bypass_t;

typedef enum {
	MCLK_GATING_IP_CLK = 0,
	ENABLE_MCLK_IP_CLK
} gen_ctrl_for_mclk_ip_t;

typedef enum {
	DISABLE_MCLK_DETECTION = 0,
	ENABLE_MCLK_DETECTION,
} gen_ctrl_for_mclk_detect_t;

typedef enum {
	DEBOUNCE_MCLK_SELECT = 0,
	DEBOUNCE_INTER_CLK_SELECT,
} gen_ctrl_for_irq_debounce_t;

typedef enum {
	SINGLE_END = 0,
	DIFFERENTIAL,
} in_1_2_ip_mode_t;

typedef union {
	struct {
		uint16_t alc5660_id : 1;
		uint16_t reserved : 15;
	};
	uint16_t bits;
} software_reset_t;


typedef union {
	struct {
		uint16_t reserved : 8;
		uint16_t spkr_vol : 6;
		uint16_t mu_spkr_vol : 1;
		uint16_t mu_spkr_port : 1;
	};
	uint16_t bits;
} speaker_op_ctrl_t;

typedef union {
	struct {
		uint16_t left_chnl : 8;
		uint16_t right_chnl : 8;
	};
	uint16_t bits;
} dac1_volume_t;


typedef union {
	struct {
		uint16_t reserved1 : 6;
		uint16_t mu_if1_dac_r : 1;
		uint16_t mu_adc_dac_r : 1;
		uint16_t reserved2 : 6;
		uint16_t mu_if1_dac_l : 1;
		uint16_t mu_adc_dac_l : 1;
	};
	uint16_t bits;
} adc_dac1_mixer_ctrl_t;

typedef union {
	struct {
		uint16_t gain_dacl1_stereo_r : 1;
		uint16_t mu_stereo_dacl1_mixr : 1;
		uint16_t reserved1 : 3;
		uint16_t gain_dacr1_stereo_r : 1;
		uint16_t mu_stereo_dacr1_mixr : 1;
		uint16_t reserved2 : 1;
		uint16_t gain_dacr1_stereo_l : 1;
		uint16_t mu_stereo_dacr1_mixl : 1;
		uint16_t reserved3 : 3;
		uint16_t gain_dacl1_stereo_l : 1;
		uint16_t mu_stereo_dacl1_mixl : 1;
		uint16_t reserved4 : 1;
	};
	uint16_t bits;
} stereo_dac1_mixer_ctrl_t;

typedef union {
	struct {
		uint16_t reserved : 12;
		uint16_t sel_if1_adc_data : 2;
		uint16_t sel_if1_dac_data : 2;
	};
	uint16_t bits;
} digi_intf_data_ctrl_t;

typedef union {
	struct {
		uint16_t reserved : 11;
		uint16_t sel_bst1_spo_src : 1;
		uint16_t sel_spkvol_spo_src : 1;
		uint16_t sel_dacl_spo_src : 1;
		uint16_t sel_dacr_spo_src : 1;
		uint16_t reserved1 : 1;
	};
	uint16_t bits;
} spkr_out_mix_ctrl_t;

typedef union {
	struct {
		uint16_t sel_i2s1_frmt : 2;
		uint16_t sel_i2s1_len : 2;
		uint16_t reserved1 : 3;
		uint16_t inv_i2s1_bclk : 1;
		uint16_t en_i2s1_in_comp : 2;
		uint16_t en_i2s1_out_comp : 2;
		uint16_t reserved2 : 3;
		uint16_t sel_i2s1_ms : 1;
	};
	uint16_t bits;
} i2s1_digi_intf_ctrl_t;

typedef union {
	struct {
		uint16_t sel_adc_osr : 2;
		uint16_t sel_dac_osr : 2;
		uint16_t reserved1 : 8;
		uint16_t sel_i2s_pre_div1 : 3;
		uint16_t sel_i2s_bclk_ms : 1;
	};
	uint16_t bits;
} adc_dac_clk_ctrl_1_t;

typedef union {
	struct {
		uint16_t sys_div_stereo_dac_fltr : 3;
		uint16_t sel_pll_pre_div : 1;
		uint16_t reserved1 : 8;
		uint16_t sel_pll_sour : 2;
		uint16_t sel_sysclk_1 : 2;
	};
	uint16_t bits;
} glbl_clk_ctrl_t;

typedef union {
	struct {
		uint16_t pll_k_code : 5;
		uint16_t reserved1 : 2;
		uint16_t pll_n_code : 9;
	};
	uint16_t bits;
} pll_ctrl_1_t;

typedef union {
	struct {
		uint16_t reserved1 : 11;
		uint16_t pll_m_bypass : 1;
		uint16_t pll_m_code : 4;
	};
	uint16_t bits;
} pll_ctrl_2_t;

typedef union {
	struct {
		uint16_t pow_clsd : 1;
		uint16_t pow_adc_r : 1;
		uint16_t pow_adc_l : 1;
		uint16_t reserved1 : 8;
		uint16_t pow_dac_r : 1;
		uint16_t pow_dac_l : 1;
		uint16_t reserved2 : 2;
		uint16_t en_i2s1 : 1;
	};
	uint16_t bits;
} pwr_mgmt_ctrl_1_t;

typedef union {
	struct {
		uint16_t reserved1 : 11;
		uint16_t pow_dac_stereo1_fltr : 1;
		uint16_t reserved2 : 3;
		uint16_t pow_adc_stereo1_fltr : 1;
	};
	uint16_t bits;
} pwr_mgmt_ctrl_2_t;

typedef union {
	struct {
		uint16_t reserved1 : 1;
		uint16_t reserved_un : 1; /* Bit-1 not defined in ALC doc */
		uint16_t pow_ldo2 : 1;
		uint16_t en_fastb2 : 1;
		uint16_t pow_vref2 : 1;
		uint16_t en_amp_hp : 1;
		uint16_t en_r_hp : 1;
		uint16_t en_l_hp : 1;
		uint16_t reserved2 : 3;
		uint16_t pow_bg_bias : 1;
		uint16_t reserved3 : 1;
		uint16_t pow_main_bias : 1;
		uint16_t en_fastb1 : 1;
		uint16_t pow_vref1 : 1;
	};
	uint16_t bits;
} pwr_mgmt_ctrl_3_t;

typedef union {
	struct {
		uint16_t reserved1 : 9;
		uint16_t pow_pll : 1;
		uint16_t pow_micbias2 : 1;
		uint16_t pow_micbias1 : 1;
		uint16_t reserved2 : 1;
		uint16_t pow_bst3 : 1;
		uint16_t pow_bst2 : 1;
		uint16_t pow_bst1 : 1;
	};
	uint16_t bits;
} pwr_mgmt_ctrl_4_t;

typedef union {
	struct {
		uint16_t reserved1 : 10;
		uint16_t pow_recmixr : 1;
		uint16_t pow_recmixl : 1;
		uint16_t reserved2 : 1;
		uint16_t pow_spkmix : 1;
		uint16_t pow_outmixr : 1;
		uint16_t pow_outmixl : 1;
	};
	uint16_t bits;
} pwr_mgmt_ctrl_5_t;

typedef union {
	struct {
		uint16_t reserved1 : 10;
		uint16_t pow_loutvolr : 1;
		uint16_t pow_loutvoll : 1;
		uint16_t reserved2 : 3;
		uint16_t pow_spkvol : 1;
	};
	uint16_t bits;
} pwr_mgmt_ctrl_6_t;

typedef union {
	struct {
		uint16_t digi_gate_ctrl : 1;
		uint16_t reserved1 : 3;
		uint16_t sel_debounce : 1;
		uint16_t mclk_detect : 1;
		uint16_t reserved2 : 10;
	};
	uint16_t bits;
} gen_ctrl_1_t;

typedef union {
	struct {
		uint16_t mu_dacl_outmixl : 1;
		uint16_t mu_dacr_outmixl : 1;
		uint16_t mu_recmixl_outmixl : 1;
		uint16_t mu_bst1_outmixl : 1;
		uint16_t mu_bst2_outmixl : 1;
		uint16_t mu_bst3_outmixl : 1;
		uint16_t reserved : 10;
	};
	uint16_t bits;
} outmixl_ctrl_3_t;

typedef union {
	struct {
		uint16_t mu_dacr_outmixr : 1;
		uint16_t mu_dacl_outmixr : 1;
		uint16_t mu_recmixr_outmixr : 1;
		uint16_t mu_bst1_outmixr : 1;
		uint16_t mu_bst2_outmixr : 1;
		uint16_t reserved : 11;
	};
	uint16_t bits;
} outmixr_ctrl_3_t;

typedef union {
	struct {
		uint16_t vol_loutr : 6;
		uint16_t mu_loutvolr_in : 1;
		uint16_t mu_lout_r : 1;
		uint16_t vol_loutl : 6;
		uint16_t mu_loutvoll_in : 1;
		uint16_t mu_lout_l : 1;
	};
	uint16_t bits;
} lineoutput_ctrl_t;

typedef union {
	struct {
		uint16_t reserved : 12;
		uint16_t gain_loutmix : 1;
		uint16_t mu_loutvol_loutmix : 1;
		uint16_t mu_dac1_loutmix : 1;
		uint16_t reserved_2 : 1;
	};
	uint16_t bits;
} lineout_mix_ctrl_t;

typedef union {
	struct {
		uint16_t sel_bst2 : 7;
		uint16_t reserved : 1;
		uint16_t sel_bst1 : 7;
		uint16_t en_in1_df : 1;
	};
	uint16_t bits;
} in_1_2_input_ctrl_t;

typedef union {
	struct {
		uint16_t mu_outmixl_recmixl : 1;
		uint16_t mu_bst1_recmixl : 1;
		uint16_t mu_bst2_recmixl : 1;
		uint16_t mu_bst3_recmixl : 1;
		uint16_t reserved : 6;
		uint16_t gain_outmixl_recmixl : 3;
		uint16_t gain_bst1_recmixl : 3;
	};
	uint16_t bits;
} recmixl_ctrl_2_t;

typedef union {
	struct {
		uint16_t reserved : 1;
		uint16_t mu_bst1_recmixr : 1;
		uint16_t mu_bst2_recmixr : 1;
		uint16_t mu_bst3_recmixr : 1;
		uint16_t reserved_2 : 6;
		uint16_t gain_outmixr_recmixr : 3;
		uint16_t gain_bst1_recmixr : 3;
	};
	uint16_t bits;
} recmixr_ctrl_2_t;

typedef union {
	struct {
		uint16_t vol_adc1_r : 7;
		uint16_t mu_adc_vol_r : 1;
		uint16_t vol_adc1_l : 7;
		uint16_t mu_adc_vol_l : 1;
	};
	uint16_t bits;
} stereo1_adc_digi_vol_ctrl_t;

typedef union {
	struct {
		uint16_t reserved : 5;
		uint16_t mu_stereo_adcr2 : 1;
		uint16_t mu_stereo_adcr1 : 1;
		uint16_t reserved_2 : 6;
		uint16_t mu_stereo_adcl2 : 1;
		uint16_t mu_stereo_adcl1 : 1;
		uint16_t reserved_3 : 1;
	};
	uint16_t bits;
} stereo1_adc_digi_mixer_ctrl_t;

typedef union {
	struct {
		uint16_t sel_vol : 4;
		uint16_t reserved_1 : 6;
		uint16_t pow_zcd : 1;
		uint16_t en_zcd_digi : 1;
		uint16_t en_o_svol : 1;
		uint16_t reserved_2 : 1;
		uint16_t en_spo_svol : 1;
		uint16_t en_softvol : 1;
	};
	uint16_t bits;
} sftvol_zcd_ctrl1_t;

typedef union {
	struct {
		uint16_t reserved_1 : 4;
		uint16_t en_zcd_recmixl : 1;
		uint16_t en_zcd_recmixr : 1;
		uint16_t en_zcd_spkmix : 1;
		uint16_t en_zcd_outmixl : 1;
		uint16_t en_zcd_outmixr : 1;
		uint16_t reserved_2 : 6;
		uint16_t en_zcd_hp : 1;
	};
	uint16_t bits;
} sftvol_zcd_ctrl2_t;

typedef union {
	struct {
		uint16_t reserved_1 : 4;
		uint16_t pow_int_clk : 1;
		uint16_t reserved_2 : 1;
		uint16_t mic2_ovcd_th_sel : 2;
		uint16_t pow_mic2_ovcd : 1;
		uint16_t mic1_ovcd_th_sel : 2;
		uint16_t pow_mic1_ovcd : 1;
		uint16_t reserved_3 : 2;
		uint16_t sel_micbias_2 : 1;
		uint16_t sel_micbias_1 : 1;
	};
	uint16_t bits;
} mic_bias_ctrl_t;

typedef union {
	struct {
		uint16_t pow_lout : 1;
		uint16_t reserved_1 : 3;
		uint16_t en_out_lout : 1;
		uint16_t reserved_2 : 11;
	};
	uint16_t bits;
} lout_amp_ctrl1_t;

typedef struct {
	uint8_t reg_addr;
	uint8_t reg_high_byte;
	uint8_t reg_low_byte;
} alc_reg_write_read_t;


#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_AUDIO_ALC5660I_H */
