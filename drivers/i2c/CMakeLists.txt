# SPDX-License-Identifier: Apache-2.0

target_sources_ifdef(CONFIG_I2C_SEDI drivers__i2c PRIVATE i2c_sedi.c)
