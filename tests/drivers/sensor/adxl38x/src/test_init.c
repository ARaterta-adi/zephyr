/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ADXL380/ADXL382 driver initialization tests (Track B, requirement group
 * SRS-F-INIT). Runs on native_sim and native_sim/native/64 against the
 * register-file SPI emulator.
 *
 * Covered here:
 *   TC-INIT-001  Device-ID check passes -> device becomes ready (SRS-F-INIT-001)
 *   TC-INIT-003  Soft reset issued (RESET register holds 0x52)   (SRS-F-INIT-003)
 *   TC-INIT-004  Devicetree config applied to registers          (SRS-F-INIT-004)
 *
 * TODO (next increment): TC-INIT-002 (D1 power-on ID retry) and the -ENODEV
 * negative path need a stateful emulator that returns wrong IDs for the first N
 * reads; add once the emulator grows a per-access counter.
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/emul.h>

#include "adxl38x_emul.h"

#define ADXL380_NODE DT_NODELABEL(adxl380)
#define ADXL382_NODE DT_NODELABEL(adxl382)

/* ADXL38X register addresses used for config read-back (datasheet / driver). */
#define REG_OP_MODE   0x26U
#define REG_DIG_EN    0x27U
#define REG_REG_RESET 0x2AU
#define REG_FIFO_CFG0 0x30U
#define REG_FIFO_CFG1 0x31U
#define REG_FILTER    0x50U
#define REG_INT0      0x5DU
#define REG_INT1      0x5EU

#define RESET_CODE       0x52U
#define OP_MODE_HP       0x0CU /* ADXL38X_DT_OP_HP */
#define RANGE_MSK        0xC0U /* OP_MODE[7:6] */
#define OP_MODE_LOW_MSK  0x0FU /* OP_MODE[3:0] */
#define HPF_CORNER_MSK   0x07U /* FILTER[2:0] */
#define INT_POL_MSK      0x80U /* INTx[7] */
/* X,Y,Z,TEMP channels enabled; streaming also sets the FIFO_EN bit (0x08). */
#if defined(CONFIG_ADXL38X_STREAM)
#define DIG_EN_XYZ_TEMP  0xF8U
#else
#define DIG_EN_XYZ_TEMP  0xF0U
#endif
#define FIFO_WATERMARK   96U   /* overlay value; low byte lands in FIFO_CFG1 */

static const struct device *const adxl380_dev = DEVICE_DT_GET(ADXL380_NODE);
static const struct device *const adxl382_dev = DEVICE_DT_GET(ADXL382_NODE);
static const struct emul *const adxl380_emul = EMUL_DT_GET(ADXL380_NODE);
static const struct emul *const adxl382_emul = EMUL_DT_GET(ADXL382_NODE);

/* TC-INIT-001: init only completes (device ready) if the device-ID check,
 * soft reset, configure and set_op_mode all succeeded against the emulator.
 */
ZTEST(adxl38x_init, test_devices_ready)
{
	zassert_true(device_is_ready(adxl380_dev), "ADXL380 device not ready");
	zassert_true(device_is_ready(adxl382_dev), "ADXL382 device not ready");
}

/* TC-INIT-003: the driver writes the soft-reset code to the RESET register. */
ZTEST(adxl38x_init, test_soft_reset_issued)
{
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, REG_REG_RESET), RESET_CODE,
		      "ADXL380 soft reset code not written");
	zassert_equal(adxl38x_emul_get_reg(adxl382_emul, REG_REG_RESET), RESET_CODE,
		      "ADXL382 soft reset code not written");
}

/* TC-INIT-004: devicetree defaults land in the right register fields.
 * ADXL380 overlay: range = 16 g (code 2), op-mode = HP (12).
 * OP_MODE = (2 << 6) | 12 = 0x8C.
 */
ZTEST(adxl38x_init, test_config_applied_adxl380)
{
	uint8_t op_mode = adxl38x_emul_get_reg(adxl380_emul, REG_OP_MODE);

	zassert_equal(op_mode & RANGE_MSK, 2U << 6, "ADXL380 range field wrong (0x%02x)", op_mode);
	zassert_equal(op_mode & OP_MODE_LOW_MSK, OP_MODE_HP, "ADXL380 op-mode field wrong (0x%02x)",
		      op_mode);
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, REG_DIG_EN), DIG_EN_XYZ_TEMP,
		      "ADXL380 channels not enabled");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, REG_FIFO_CFG1), FIFO_WATERMARK,
		      "ADXL380 FIFO watermark low byte wrong");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, REG_FIFO_CFG0) & 0x01U, 0U,
		      "ADXL380 FIFO watermark high bit set unexpectedly");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, REG_FILTER) & HPF_CORNER_MSK, 0U,
		      "ADXL380 HPF corner should be off");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, REG_INT0) & INT_POL_MSK, 0U,
		      "ADXL380 INT0 polarity should be active-high");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, REG_INT1) & INT_POL_MSK, 0U,
		      "ADXL380 INT1 polarity should be active-high");
}

/* ADXL382 overlay: range = 30 g (code 1), op-mode = HP (12).
 * OP_MODE = (1 << 6) | 12 = 0x4C. Sensitivity/range table differs per part.
 */
ZTEST(adxl38x_init, test_config_applied_adxl382)
{
	uint8_t op_mode = adxl38x_emul_get_reg(adxl382_emul, REG_OP_MODE);

	zassert_equal(op_mode & RANGE_MSK, 1U << 6, "ADXL382 range field wrong (0x%02x)", op_mode);
	zassert_equal(op_mode & OP_MODE_LOW_MSK, OP_MODE_HP, "ADXL382 op-mode field wrong (0x%02x)",
		      op_mode);
	zassert_equal(adxl38x_emul_get_reg(adxl382_emul, REG_DIG_EN), DIG_EN_XYZ_TEMP,
		      "ADXL382 channels not enabled");
}

ZTEST_SUITE(adxl38x_init, NULL, NULL, NULL, NULL, NULL);
