/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Register-file SPI emulator backend for the ADXL380/ADXL382 Zephyr driver.
 *
 * The driver frames every access as a 1-byte command (addr << 1 | R/W) followed
 * by the data payload (see adxl38x_reg_access_spi()). This backend decodes that
 * framing against a flat 128-byte register array seeded with the device reset
 * values the driver verifies at init (DEVID_AD=0xAD, PART_ID=0x17,
 * DEVID_MST=0x1D). It binds to both adi,adxl380 and adi,adxl382 instances.
 */

#define DT_DRV_COMPAT adi_adxl380

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi_emul.h>
#include <zephyr/logging/log.h>

#include "adxl38x_emul.h"

LOG_MODULE_REGISTER(adxl38x_emul, CONFIG_SENSOR_LOG_LEVEL);

#define ADXL38X_EMUL_NUM_REGS 0x80U

/* Register addresses and reset values the driver checks during init. */
#define ADXL38X_EMUL_REG_DEVID_AD  0x00U
#define ADXL38X_EMUL_REG_DEVID_MST 0x01U
#define ADXL38X_EMUL_REG_PART_ID   0x02U
#define ADXL38X_EMUL_VAL_DEVID_AD  0xADU
#define ADXL38X_EMUL_VAL_DEVID_MST 0x1DU
#define ADXL38X_EMUL_VAL_PART_ID   0x17U

struct adxl38x_emul_data {
	uint8_t reg[ADXL38X_EMUL_NUM_REGS];
};

uint8_t adxl38x_emul_get_reg(const struct emul *target, uint8_t reg)
{
	struct adxl38x_emul_data *data = target->data;

	if (reg >= ADXL38X_EMUL_NUM_REGS) {
		return 0U;
	}

	return data->reg[reg];
}

void adxl38x_emul_set_reg(const struct emul *target, uint8_t reg, uint8_t val)
{
	struct adxl38x_emul_data *data = target->data;

	if (reg < ADXL38X_EMUL_NUM_REGS) {
		data->reg[reg] = val;
	}
}

static int adxl38x_emul_io(const struct emul *target, const struct spi_config *config,
			   const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	struct adxl38x_emul_data *data = target->data;
	const struct spi_buf *cmd_buf;
	const struct spi_buf *txd;
	const struct spi_buf *rxd;
	uint8_t cmd;
	uint8_t reg;
	bool read;

	ARG_UNUSED(config);

	/* The driver always issues: [cmd byte][payload] on TX, [skip][payload] on RX. */
	if (tx_bufs == NULL || tx_bufs->count < 2) {
		LOG_ERR("unexpected tx_bufs");
		return -EIO;
	}

	cmd_buf = &tx_bufs->buffers[0];
	txd = &tx_bufs->buffers[1];
	rxd = (rx_bufs != NULL) ? &rx_bufs->buffers[1] : NULL;

	if (cmd_buf->buf == NULL || cmd_buf->len != 1U) {
		LOG_ERR("unexpected command buffer");
		return -EIO;
	}

	cmd = ((const uint8_t *)cmd_buf->buf)[0];
	reg = cmd >> 1;
	read = (cmd & 0x01U) != 0U;

	if (read) {
		if (rxd == NULL || (rxd->len > 0U && rxd->buf == NULL)) {
			LOG_ERR("read without rx buffer");
			return -EIO;
		}
		for (size_t i = 0; i < rxd->len; i++) {
			uint8_t addr = reg + (uint8_t)i;

			((uint8_t *)rxd->buf)[i] =
				(addr < ADXL38X_EMUL_NUM_REGS) ? data->reg[addr] : 0U;
		}
	} else {
		if (txd->len == 0U) {
			return 0;
		}
		if (txd->buf == NULL) {
			LOG_ERR("write without tx payload");
			return -EIO;
		}
		for (size_t i = 0; i < txd->len; i++) {
			uint8_t addr = reg + (uint8_t)i;

			if (addr < ADXL38X_EMUL_NUM_REGS) {
				data->reg[addr] = ((const uint8_t *)txd->buf)[i];
			}
		}
	}

	return 0;
}

static const struct spi_emul_api adxl38x_emul_api = {
	.io = adxl38x_emul_io,
};

static int adxl38x_emul_init(const struct emul *target, const struct device *parent)
{
	struct adxl38x_emul_data *data = target->data;

	ARG_UNUSED(parent);

	memset(data->reg, 0, sizeof(data->reg));
	data->reg[ADXL38X_EMUL_REG_DEVID_AD] = ADXL38X_EMUL_VAL_DEVID_AD;
	data->reg[ADXL38X_EMUL_REG_DEVID_MST] = ADXL38X_EMUL_VAL_DEVID_MST;
	data->reg[ADXL38X_EMUL_REG_PART_ID] = ADXL38X_EMUL_VAL_PART_ID;

	return 0;
}

#define ADXL38X_EMUL_DEFINE(n, pfx)                                                                 \
	static struct adxl38x_emul_data adxl38x_emul_data_##pfx##_##n;                              \
	EMUL_DT_INST_DEFINE(n, adxl38x_emul_init, &adxl38x_emul_data_##pfx##_##n, NULL,             \
			    &adxl38x_emul_api, NULL)

/* adi,adxl380 instances */
#define ADXL380_EMUL_DEFINE(n) ADXL38X_EMUL_DEFINE(n, adxl380)
DT_INST_FOREACH_STATUS_OKAY(ADXL380_EMUL_DEFINE)

/* adi,adxl382 instances */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT          adi_adxl382
#define ADXL382_EMUL_DEFINE(n) ADXL38X_EMUL_DEFINE(n, adxl382)
DT_INST_FOREACH_STATUS_OKAY(ADXL382_EMUL_DEFINE)
