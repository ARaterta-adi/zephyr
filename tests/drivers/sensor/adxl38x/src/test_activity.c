/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ADXL380/ADXL382 activity-detection tests (Track B, requirement group
 * SRS-F-ACT). Runs on native_sim and native_sim/native/64 against the
 * register-file SPI emulator.
 *
 * Covered here:
 *   TC-ACT-001  activity/inactivity threshold (m/s^2 -> SAR code) (SRS-F-ACT-001)
 *   TC-ACT-002  activity time (us -> 500 us ticks)                (SRS-F-ACT-002)
 *   TC-ACT-003  activity axis enable X/Y/Z                        (SRS-F-ACT-003)
 *
 * Threshold codes depend on the full-scale range, so the threshold test pins the
 * range to 16 g first. code = micro_ms2 * 2048 / (SENSOR_G * fsr_g); at 16 g a
 * 1 g threshold is exactly 128 codes and 8 g is exactly 1024 codes.
 */

#include <errno.h>

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>

#include "adxl38x.h"
#include "adxl38x_emul.h"

#define ADXL380_NODE DT_NODELABEL(adxl380)

static const struct device *const adxl380_dev = DEVICE_DT_GET(ADXL380_NODE);
static const struct emul *const adxl380_emul = EMUL_DT_GET(ADXL380_NODE);

static int attr_set(const struct device *dev, enum sensor_attribute attr, int32_t val1,
		    int32_t val2)
{
	struct sensor_value v = {.val1 = val1, .val2 = val2};

	return sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, attr, &v);
}

/* TC-ACT-001: threshold in m/s^2 converts to an 11-bit SAR code split across
 * THRESH_*_H[2:0] and THRESH_*_L[7:0]; negative and over-range are rejected.
 */
ZTEST(adxl38x_activity, test_activity_threshold)
{
	/* Pin range to 16 g so the code arithmetic is deterministic. */
	zassert_ok(attr_set(adxl380_dev, SENSOR_ATTR_FULL_SCALE, ADXL380_RANGE_16G_FS, 0),
		   "range set failed");

	/* 1 g = 9.806650 m/s^2 -> code 128 (0x080): H = 0, L = 0x80. */
	zassert_ok(attr_set(adxl380_dev, SENSOR_ATTR_ADXL38X_ACTIVITY_THRESHOLD, 9, 806650),
		   "1 g activity threshold set failed");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_THRESH_ACT_H), 0,
		      "THRESH_ACT_H != 0");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_THRESH_ACT_L), 0x80,
		      "THRESH_ACT_L != 0x80");

	/* 8 g = 78.453200 m/s^2 -> code 1024 (0x400): H = 0x04, L = 0x00. */
	zassert_ok(attr_set(adxl380_dev, SENSOR_ATTR_ADXL38X_ACTIVITY_THRESHOLD, 78, 453200),
		   "8 g activity threshold set failed");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_THRESH_ACT_H), 0x04,
		      "THRESH_ACT_H != 0x04");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_THRESH_ACT_L), 0x00,
		      "THRESH_ACT_L != 0x00");

	/* Inactivity threshold uses its own register pair. */
	zassert_ok(attr_set(adxl380_dev, SENSOR_ATTR_ADXL38X_INACTIVITY_THRESHOLD, 9, 806650),
		   "1 g inactivity threshold set failed");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_THRESH_INACT_H), 0,
		      "THRESH_INACT_H != 0");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_THRESH_INACT_L), 0x80,
		      "THRESH_INACT_L != 0x80");

	/* Negative threshold and a code > 2047 (16 g -> 2048) are rejected. */
	zassert_equal(attr_set(adxl380_dev, SENSOR_ATTR_ADXL38X_ACTIVITY_THRESHOLD, -1, 0), -EINVAL,
		      "negative threshold should fail");
	zassert_equal(attr_set(adxl380_dev, SENSOR_ATTR_ADXL38X_ACTIVITY_THRESHOLD, 156, 906400),
		      -EINVAL, "over-range threshold should fail");
}

/* TC-ACT-002: activity time in microseconds converts to 500 us ticks across the
 * 24-bit TIME_ACT register triple; negative and over-range are rejected.
 */
ZTEST(adxl38x_activity, test_activity_time)
{
	/* 500000 us / 500 = 1000 ticks (0x0003E8): H = 0, M = 0x03, L = 0xE8. */
	zassert_ok(attr_set(adxl380_dev, SENSOR_ATTR_ADXL38X_ACTIVITY_TIME, 0, 500000),
		   "activity time set failed");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_TIME_ACT_H), 0x00,
		      "TIME_ACT_H != 0");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_TIME_ACT_M), 0x03,
		      "TIME_ACT_M != 0x03");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_TIME_ACT_L), 0xE8,
		      "TIME_ACT_L != 0xE8");

	/* Negative time rejected. */
	zassert_equal(attr_set(adxl380_dev, SENSOR_ATTR_ADXL38X_ACTIVITY_TIME, -1, 0), -EINVAL,
		      "negative activity time should fail");

	/* 8388608000 us / 500 = 16777216 ticks > 24-bit max -> rejected. */
	zassert_equal(attr_set(adxl380_dev, SENSOR_ATTR_ADXL38X_ACTIVITY_TIME, 8388, 608000),
		      -EINVAL, "over-range activity time should fail");
}

/* TC-ACT-003: each axis-enable attribute toggles its SNSR_AXIS_EN bit. */
ZTEST(adxl38x_activity, test_activity_axis_enable)
{
	static const struct {
		enum sensor_attribute attr;
		uint8_t bit;
	} axes[] = {
		{SENSOR_ATTR_ADXL38X_ACTIVITY_ENABLE_X, BIT(0)},
		{SENSOR_ATTR_ADXL38X_ACTIVITY_ENABLE_Y, BIT(1)},
		{SENSOR_ATTR_ADXL38X_ACTIVITY_ENABLE_Z, BIT(2)},
	};

	for (size_t i = 0; i < ARRAY_SIZE(axes); i++) {
		zassert_ok(attr_set(adxl380_dev, axes[i].attr, 1, 0), "axis %u enable failed", i);
		zassert_true((adxl38x_emul_get_reg(adxl380_emul, ADXL38X_SNSR_AXIS_EN) & axes[i].bit) !=
				     0,
			     "axis %u enable bit not set", i);

		zassert_ok(attr_set(adxl380_dev, axes[i].attr, 0, 0), "axis %u disable failed", i);
		zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_SNSR_AXIS_EN) & axes[i].bit, 0,
			      "axis %u enable bit not cleared", i);
	}
}

ZTEST_SUITE(adxl38x_activity, NULL, NULL, NULL, NULL, NULL);
