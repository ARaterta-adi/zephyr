/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ADXL380/ADXL382 attribute tests (Track B, requirement group SRS-F-ATTR).
 * Runs on native_sim and native_sim/native/64 against the register-file SPI
 * emulator, driving the public sensor_attr_set()/sensor_attr_get() API and
 * checking both the returned values and the register bytes the driver wrote.
 *
 * Covered here:
 *   TC-ATTR-001  ODR get/set in DSM mode (sinc_rate / dec2x_bypass) (SRS-F-ATTR-001)
 *   TC-ATTR-002  ODR rejected in SAR mode                           (SRS-F-ATTR-002)
 *   TC-ATTR-003  double-speed ODR multiplier (ADXL380 only)         (SRS-F-ATTR-003)
 *   TC-ATTR-004  full-scale range get/set, per-part validation      (SRS-F-ATTR-004)
 *   TC-ATTR-005  OP_MODE get/set                                    (SRS-F-ATTR-005)
 *   TC-ATTR-006  FIFO watermark                                     (SRS-F-ATTR-006)
 *   TC-ATTR-007  HPF corner                                         (SRS-F-ATTR-007)
 *   TC-ATTR-008  INT polarity                                       (SRS-F-ATTR-008)
 *   TC-ATTR-009  attr_set standby-wrap restore (incl. error path)   (SRS-F-ATTR-009)
 *
 * Each test establishes its own preconditions (op-mode / range), because the
 * device and emulator state persist across tests in a single ztest run.
 *
 * Note on TC-ATTR-009: this exercises the deterministic half of the finding --
 * that attr_set restores the operating mode after both a successful and a failed
 * set, instead of leaving the device stuck in standby. The BUG-002 data race
 * (reconfigure concurrent with an active FIFO stream) is not deterministically
 * reproducible in a single-threaded unit test and is covered on the HW track.
 */

#include <errno.h>

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>

/* Private driver header: register addresses/masks, op-mode enum, DT macros and
 * the SENSOR_ATTR_ADXL38X_* attribute enum (included transitively).
 */
#include "adxl38x.h"
#include "adxl38x_emul.h"

#define ADXL380_NODE    DT_NODELABEL(adxl380)
#define ADXL382_NODE    DT_NODELABEL(adxl382)
#define ADXL380_DS_NODE DT_NODELABEL(adxl380_ds)

static const struct device *const adxl380_dev = DEVICE_DT_GET(ADXL380_NODE);
static const struct device *const adxl382_dev = DEVICE_DT_GET(ADXL382_NODE);
static const struct device *const adxl380_ds_dev = DEVICE_DT_GET(ADXL380_DS_NODE);
static const struct emul *const adxl380_emul = EMUL_DT_GET(ADXL380_NODE);

static int attr_set_i(const struct device *dev, enum sensor_attribute attr, int32_t val1)
{
	struct sensor_value v = {.val1 = val1, .val2 = 0};

	return sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, attr, &v);
}

static int32_t attr_get_i(const struct device *dev, enum sensor_attribute attr)
{
	struct sensor_value v = {0};

	zassert_ok(sensor_attr_get(dev, SENSOR_CHAN_ACCEL_XYZ, attr, &v), "attr_get %d failed", attr);
	return v.val1;
}

static void set_op_mode(const struct device *dev, int32_t mode)
{
	zassert_ok(attr_set_i(dev, SENSOR_ATTR_ADXL38X_OP_MODE, mode), "op-mode %d set failed", mode);
}

/* TC-ATTR-001: in a DSM mode the ODR maps to the sinc_rate / dec2x_bypass bits
 * of TRIG_CFG, and get returns the effective rate. ADXL380 HP base = 8000 Hz.
 */
ZTEST(adxl38x_attr, test_odr_dsm_roundtrip)
{
	uint8_t trig;

	set_op_mode(adxl380_dev, ADXL38X_HP_MODE);

	/* 16000 Hz = base x2 -> DEC2X_BYPASS set, SINC_RATE clear. */
	zassert_ok(attr_set_i(adxl380_dev, SENSOR_ATTR_SAMPLING_FREQUENCY, 16000),
		   "set 16000 failed");
	trig = adxl38x_emul_get_reg(adxl380_emul, ADXL38X_TRIG_CFG);
	zassert_true((trig & ADXL38X_TRIG_CFG_DEC_2X_BYPASS_MSK) != 0, "DEC2X not set (0x%02x)", trig);
	zassert_true((trig & ADXL38X_TRIG_CFG_SINC_RATE_MSK) == 0, "SINC set (0x%02x)", trig);
	zassert_equal(attr_get_i(adxl380_dev, SENSOR_ATTR_SAMPLING_FREQUENCY), 16000, "get != 16000");

	/* 32000 Hz = base x4 -> SINC_RATE set, DEC2X_BYPASS clear. */
	zassert_ok(attr_set_i(adxl380_dev, SENSOR_ATTR_SAMPLING_FREQUENCY, 32000),
		   "set 32000 failed");
	trig = adxl38x_emul_get_reg(adxl380_emul, ADXL38X_TRIG_CFG);
	zassert_true((trig & ADXL38X_TRIG_CFG_SINC_RATE_MSK) != 0, "SINC not set (0x%02x)", trig);
	zassert_true((trig & ADXL38X_TRIG_CFG_DEC_2X_BYPASS_MSK) == 0, "DEC2X set (0x%02x)", trig);
	zassert_equal(attr_get_i(adxl380_dev, SENSOR_ATTR_SAMPLING_FREQUENCY), 32000, "get != 32000");

	/* 8000 Hz = base -> both bits clear. */
	zassert_ok(attr_set_i(adxl380_dev, SENSOR_ATTR_SAMPLING_FREQUENCY, 8000), "set 8000 failed");
	trig = adxl38x_emul_get_reg(adxl380_emul, ADXL38X_TRIG_CFG);
	zassert_true((trig & (ADXL38X_TRIG_CFG_SINC_RATE_MSK | ADXL38X_TRIG_CFG_DEC_2X_BYPASS_MSK)) ==
			     0,
		     "ODR bits not cleared (0x%02x)", trig);
	zassert_equal(attr_get_i(adxl380_dev, SENSOR_ATTR_SAMPLING_FREQUENCY), 8000, "get != 8000");

	/* Unsupported rate for the mode. */
	zassert_equal(attr_set_i(adxl380_dev, SENSOR_ATTR_SAMPLING_FREQUENCY, 9999), -EINVAL,
		      "9999 Hz should be rejected");
}

/* TC-ATTR-002: ODR is fixed in a SAR mode, so set must report -ENOTSUP. */
ZTEST(adxl38x_attr, test_odr_rejected_in_sar)
{
	set_op_mode(adxl380_dev, ADXL38X_ULTRA_LP_MODE);
	zassert_equal(attr_set_i(adxl380_dev, SENSOR_ATTR_SAMPLING_FREQUENCY, 50), -ENOTSUP,
		      "ODR set in SAR mode should be -ENOTSUP");

	/* Restore a DSM mode for later tests. */
	set_op_mode(adxl380_dev, ADXL38X_HP_MODE);
}

/* TC-ATTR-003: double-speed doubles the effective ODR on the ADXL380 only. */
ZTEST(adxl38x_attr, test_double_speed_odr)
{
	zassert_true(device_is_ready(adxl380_ds_dev), "double-speed node not ready");

	/* Plain ADXL380 at HP base = 8000 Hz. */
	set_op_mode(adxl380_dev, ADXL38X_HP_MODE);
	zassert_ok(attr_set_i(adxl380_dev, SENSOR_ATTR_SAMPLING_FREQUENCY, 8000), "base set failed");
	zassert_equal(attr_get_i(adxl380_dev, SENSOR_ATTR_SAMPLING_FREQUENCY), 8000,
		      "plain ADXL380 ODR != 8000");

	/* Double-speed ADXL380 at HP = 8000 x2 = 16000 Hz. */
	zassert_equal(attr_get_i(adxl380_ds_dev, SENSOR_ATTR_SAMPLING_FREQUENCY), 16000,
		      "double-speed ADXL380 ODR != 16000");

	/* ADXL382 HP base is 16000 Hz; the multiplier never applies to it. */
	set_op_mode(adxl382_dev, ADXL38X_HP_MODE);
	zassert_equal(attr_get_i(adxl382_dev, SENSOR_ATTR_SAMPLING_FREQUENCY), 16000,
		      "ADXL382 ODR != 16000");
}

/* TC-ATTR-004: full-scale range round-trips per part; invalid g is rejected. */
ZTEST(adxl38x_attr, test_range_roundtrip)
{
	zassert_ok(attr_set_i(adxl380_dev, SENSOR_ATTR_FULL_SCALE, ADXL380_RANGE_8G_FS),
		   "380 8g set failed");
	zassert_equal(attr_get_i(adxl380_dev, SENSOR_ATTR_FULL_SCALE), ADXL380_RANGE_8G_FS,
		      "380 range get != 8g");
	zassert_ok(attr_set_i(adxl380_dev, SENSOR_ATTR_FULL_SCALE, ADXL380_RANGE_16G_FS),
		   "380 16g set failed");
	zassert_equal(attr_get_i(adxl380_dev, SENSOR_ATTR_FULL_SCALE), ADXL380_RANGE_16G_FS,
		      "380 range get != 16g");
	zassert_equal(attr_set_i(adxl380_dev, SENSOR_ATTR_FULL_SCALE, 5), -EINVAL,
		      "invalid 380 range should fail");

	zassert_ok(attr_set_i(adxl382_dev, SENSOR_ATTR_FULL_SCALE, ADXL382_RANGE_30G_FS),
		   "382 30g set failed");
	zassert_equal(attr_get_i(adxl382_dev, SENSOR_ATTR_FULL_SCALE), ADXL382_RANGE_30G_FS,
		      "382 range get != 30g");
	zassert_equal(attr_set_i(adxl382_dev, SENSOR_ATTR_FULL_SCALE, 10), -EINVAL,
		      "invalid 382 range should fail");
}

/* TC-ATTR-005: operating mode round-trips; an undefined mode is rejected. */
ZTEST(adxl38x_attr, test_op_mode_roundtrip)
{
	set_op_mode(adxl380_dev, ADXL38X_HP_MODE);
	zassert_equal(attr_get_i(adxl380_dev, SENSOR_ATTR_ADXL38X_OP_MODE), ADXL38X_HP_MODE,
		      "op-mode get != HP");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_OP_MODE) & ADXL38X_OP_MODE_MSK,
		      ADXL38X_HP_MODE, "OP_MODE register field != HP");

	set_op_mode(adxl380_dev, ADXL38X_ULTRA_LP_MODE);
	zassert_equal(attr_get_i(adxl380_dev, SENSOR_ATTR_ADXL38X_OP_MODE), ADXL38X_ULTRA_LP_MODE,
		      "op-mode get != ULP");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_OP_MODE) & ADXL38X_OP_MODE_MSK,
		      ADXL38X_ULTRA_LP_MODE, "OP_MODE register field != ULP");

	/* 5 is not a defined op-mode (reserved gap in the enum). */
	zassert_equal(attr_set_i(adxl380_dev, SENSOR_ATTR_ADXL38X_OP_MODE, 5), -EINVAL,
		      "undefined op-mode should fail");

	set_op_mode(adxl380_dev, ADXL38X_HP_MODE);
}

/* TC-ATTR-006: FIFO watermark splits across FIFO_CFG0[8] / FIFO_CFG1[7:0]; the
 * value must be non-zero, <= 320 and a multiple of 4.
 */
ZTEST(adxl38x_attr, test_fifo_watermark)
{
	zassert_ok(attr_set_i(adxl380_dev, SENSOR_ATTR_ADXL38X_FIFO_WATERMARK, 96),
		   "watermark 96 set failed");
	zassert_equal(attr_get_i(adxl380_dev, SENSOR_ATTR_ADXL38X_FIFO_WATERMARK), 96,
		      "watermark get != 96");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_FIFO_CFG1), 96,
		      "FIFO_CFG1 low byte != 96");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_FIFO_CFG0) &
			      ADXL38X_FIFO_CFG0_SAMPLES_8_MSK,
		      0, "FIFO_CFG0 high bit set unexpectedly");

	/* 320 = 0x140: high bit in FIFO_CFG0, 0x40 in FIFO_CFG1. */
	zassert_ok(attr_set_i(adxl380_dev, SENSOR_ATTR_ADXL38X_FIFO_WATERMARK, 320),
		   "watermark 320 set failed");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_FIFO_CFG1), 0x40,
		      "FIFO_CFG1 low byte != 0x40");
	zassert_true((adxl38x_emul_get_reg(adxl380_emul, ADXL38X_FIFO_CFG0) &
		      ADXL38X_FIFO_CFG0_SAMPLES_8_MSK) != 0,
		     "FIFO_CFG0 high bit not set");

	zassert_equal(attr_set_i(adxl380_dev, SENSOR_ATTR_ADXL38X_FIFO_WATERMARK, 0), -EINVAL,
		      "watermark 0 should fail");
	zassert_equal(attr_set_i(adxl380_dev, SENSOR_ATTR_ADXL38X_FIFO_WATERMARK, 324), -EINVAL,
		      "watermark > 320 should fail");
	zassert_equal(attr_set_i(adxl380_dev, SENSOR_ATTR_ADXL38X_FIFO_WATERMARK, 97), -EINVAL,
		      "watermark not multiple of 4 should fail");
}

/* TC-ATTR-007: HPF corner round-trips; out-of-range is rejected. */
ZTEST(adxl38x_attr, test_hpf_corner)
{
	zassert_ok(attr_set_i(adxl380_dev, SENSOR_ATTR_ADXL38X_HPF_CORNER, ADXL38X_DT_HPF_1_5545e_4),
		   "hpf set failed");
	zassert_equal(attr_get_i(adxl380_dev, SENSOR_ATTR_ADXL38X_HPF_CORNER),
		      ADXL38X_DT_HPF_1_5545e_4, "hpf get mismatch");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_FILTER) &
			      ADXL38X_FILTER_HPF_CORNER_MSK,
		      ADXL38X_DT_HPF_1_5545e_4, "FILTER HPF field mismatch");

	zassert_ok(attr_set_i(adxl380_dev, SENSOR_ATTR_ADXL38X_HPF_CORNER, ADXL38X_DT_HPF_OFF),
		   "hpf off set failed");
	zassert_equal(attr_get_i(adxl380_dev, SENSOR_ATTR_ADXL38X_HPF_CORNER), ADXL38X_DT_HPF_OFF,
		      "hpf get != off");

	zassert_equal(attr_set_i(adxl380_dev, SENSOR_ATTR_ADXL38X_HPF_CORNER, 7), -EINVAL,
		      "hpf corner > max should fail");
	zassert_equal(attr_set_i(adxl380_dev, SENSOR_ATTR_ADXL38X_HPF_CORNER, -1), -EINVAL,
		      "negative hpf corner should fail");
}

/* TC-ATTR-008: interrupt polarity updates both INT0 and INT1 pad registers. */
ZTEST(adxl38x_attr, test_int_pol)
{
	zassert_ok(attr_set_i(adxl380_dev, SENSOR_ATTR_ADXL38X_INT_POL,
			      ADXL38X_DT_INT_POL_ACTIVE_LOW),
		   "int-pol low set failed");
	zassert_equal(attr_get_i(adxl380_dev, SENSOR_ATTR_ADXL38X_INT_POL),
		      ADXL38X_DT_INT_POL_ACTIVE_LOW, "int-pol get != low");
	zassert_true((adxl38x_emul_get_reg(adxl380_emul, ADXL38X_INT0) & ADXL38X_INT_POL_MSK) != 0,
		     "INT0 polarity bit not set");
	zassert_true((adxl38x_emul_get_reg(adxl380_emul, ADXL38X_INT1) & ADXL38X_INT_POL_MSK) != 0,
		     "INT1 polarity bit not set");

	zassert_ok(attr_set_i(adxl380_dev, SENSOR_ATTR_ADXL38X_INT_POL,
			      ADXL38X_DT_INT_POL_ACTIVE_HIGH),
		   "int-pol high set failed");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_INT0) & ADXL38X_INT_POL_MSK, 0,
		      "INT0 polarity bit not cleared");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_INT1) & ADXL38X_INT_POL_MSK, 0,
		      "INT1 polarity bit not cleared");
}

/* TC-ATTR-009: attr_set enters standby, applies the change, and restores the
 * operating mode -- on both the success and the error path. A regression would
 * leave the device stuck in standby.
 */
ZTEST(adxl38x_attr, test_attr_set_restores_op_mode)
{
	set_op_mode(adxl380_dev, ADXL38X_HP_MODE);

	/* Successful set: op-mode must be back to HP, not standby. */
	zassert_ok(attr_set_i(adxl380_dev, SENSOR_ATTR_ADXL38X_HPF_CORNER, ADXL38X_DT_HPF_OFF),
		   "hpf set failed");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_OP_MODE) & ADXL38X_OP_MODE_MSK,
		      ADXL38X_HP_MODE, "op-mode not restored after successful set");

	/* Error path: an invalid value still restores HP. */
	zassert_equal(attr_set_i(adxl380_dev, SENSOR_ATTR_ADXL38X_HPF_CORNER, 7), -EINVAL,
		      "invalid hpf should fail");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_OP_MODE) & ADXL38X_OP_MODE_MSK,
		      ADXL38X_HP_MODE, "op-mode not restored after failed set");
	zassert_equal(attr_get_i(adxl380_dev, SENSOR_ATTR_ADXL38X_OP_MODE), ADXL38X_HP_MODE,
		      "cached op-mode not restored after failed set");
}

/* Restore the devicetree defaults the tests above changed, so the init
 * config-applied checks see the post-boot configuration regardless of the order
 * ztest runs the suites in.
 */
static void attr_teardown(void *fixture)
{
	struct sensor_value v = {.val2 = 0};

	ARG_UNUSED(fixture);

	v.val1 = ADXL380_RANGE_16G_FS;
	(void)sensor_attr_set(adxl380_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &v);
	v.val1 = 96;
	(void)sensor_attr_set(adxl380_dev, SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_ADXL38X_FIFO_WATERMARK, &v);
	v.val1 = ADXL38X_DT_HPF_OFF;
	(void)sensor_attr_set(adxl380_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_ADXL38X_HPF_CORNER, &v);
	v.val1 = ADXL38X_DT_INT_POL_ACTIVE_HIGH;
	(void)sensor_attr_set(adxl380_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_ADXL38X_INT_POL, &v);
	v.val1 = ADXL38X_HP_MODE;
	(void)sensor_attr_set(adxl380_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_ADXL38X_OP_MODE, &v);

	v.val1 = ADXL382_RANGE_30G_FS;
	(void)sensor_attr_set(adxl382_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &v);
	v.val1 = ADXL38X_HP_MODE;
	(void)sensor_attr_set(adxl382_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_ADXL38X_OP_MODE, &v);
}

ZTEST_SUITE(adxl38x_attr, NULL, NULL, NULL, NULL, attr_teardown);
