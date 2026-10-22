/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ADXL380/ADXL382 conversion tests (Track B, requirement group SRS-F-DATA).
 * Runs on native_sim and native_sim/native/64 against the register-file SPI
 * emulator, exercising the synchronous sensor_sample_fetch()/channel_get() path.
 *
 * Covered here:
 *   TC-DATA-002  accel conversion by range x chip (sensitivity) (SRS-F-DATA-002)
 *   TC-DATA-003  die-temperature conversion                     (SRS-F-DATA-003)
 *
 * Expected values are kept independent of the driver's own arithmetic by seeding
 * a raw sample that is an exact multiple of the per-range sensitivity (LSB/g):
 * raw = k * sensitivity therefore converts to exactly k * SENSOR_G micro-m/s^2,
 * which is hand-verifiable (+1 g = {9, 806650}). A wrong sensitivity table (for
 * example the ADXL380 table applied to the ADXL382) would not yield a clean g.
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>

/* Private driver header: data-register addresses and sensitivity/temp constants. */
#include "adxl38x.h"
#include "adxl38x_emul.h"

#define ADXL380_NODE DT_NODELABEL(adxl380)
#define ADXL382_NODE DT_NODELABEL(adxl382)

static const struct device *const adxl380_dev = DEVICE_DT_GET(ADXL380_NODE);
static const struct device *const adxl382_dev = DEVICE_DT_GET(ADXL382_NODE);
static const struct emul *const adxl380_emul = EMUL_DT_GET(ADXL380_NODE);
static const struct emul *const adxl382_emul = EMUL_DT_GET(ADXL382_NODE);

/* Seed the X/Y/Z data registers (big-endian 16-bit, high byte first). */
static void seed_accel(const struct emul *e, int16_t x, int16_t y, int16_t z)
{
	adxl38x_emul_set_reg(e, ADXL38X_XDATA_H, (uint8_t)((uint16_t)x >> 8));
	adxl38x_emul_set_reg(e, ADXL38X_XDATA_L, (uint8_t)((uint16_t)x & 0xFFU));
	adxl38x_emul_set_reg(e, ADXL38X_YDATA_H, (uint8_t)((uint16_t)y >> 8));
	adxl38x_emul_set_reg(e, ADXL38X_YDATA_L, (uint8_t)((uint16_t)y & 0xFFU));
	adxl38x_emul_set_reg(e, ADXL38X_ZDATA_H, (uint8_t)((uint16_t)z >> 8));
	adxl38x_emul_set_reg(e, ADXL38X_ZDATA_L, (uint8_t)((uint16_t)z & 0xFFU));
}

/* Seed the 12-bit temperature data: TDATA_H holds bits [11:4], TDATA_L[7:4]
 * holds bits [3:0] (see ADXL38X_TEMP_OUTPUT_TDATA_L_MSK).
 */
static void seed_temp(const struct emul *e, uint16_t raw12)
{
	adxl38x_emul_set_reg(e, ADXL38X_TDATA_H, (uint8_t)((raw12 >> 4) & 0xFFU));
	adxl38x_emul_set_reg(e, ADXL38X_TDATA_L, (uint8_t)((raw12 & 0x0FU) << 4));
}

static void check_ms2(const struct sensor_value *v, int64_t micro, const char *tag)
{
	int32_t want1 = (int32_t)(micro / ADXL38X_MICRO_PER_UNIT);
	int32_t want2 = (int32_t)(micro % ADXL38X_MICRO_PER_UNIT);

	zassert_equal(v->val1, want1, "%s: val1 %d != %d", tag, v->val1, want1);
	zassert_equal(v->val2, want2, "%s: val2 %d != %d", tag, v->val2, want2);
}

struct range_case {
	int32_t fs;   /* SENSOR_ATTR_FULL_SCALE value in g */
	int16_t sens; /* sensitivity in LSB/g for that range */
};

/* For each range: set full scale, seed +1 g / -1 g / +2 g (exact multiples of
 * the sensitivity), fetch, and check the converted XYZ values.
 */
static void run_range_cases(const struct device *dev, const struct emul *emul,
			    const struct range_case *cases, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		struct sensor_value fs = {.val1 = cases[i].fs, .val2 = 0};
		int16_t sens = cases[i].sens;
		struct sensor_value v[3];

		zassert_ok(sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &fs),
			   "FS=%d g set failed", cases[i].fs);

		seed_accel(emul, sens, (int16_t)-sens, (int16_t)(2 * sens));

		zassert_ok(sensor_sample_fetch(dev), "FS=%d g fetch failed", cases[i].fs);
		zassert_ok(sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, v),
			   "FS=%d g get failed", cases[i].fs);

		check_ms2(&v[0], (int64_t)SENSOR_G, "X(+1g)");
		check_ms2(&v[1], -(int64_t)SENSOR_G, "Y(-1g)");
		check_ms2(&v[2], 2 * (int64_t)SENSOR_G, "Z(+2g)");
	}
}

/* TC-DATA-002: ADXL380 conversion across its three ranges. */
ZTEST(adxl38x_convert, test_accel_convert_adxl380)
{
	static const struct range_case ranges[] = {
		{ADXL380_RANGE_4G_FS, ADXL380_SENSITIVITY_4G},
		{ADXL380_RANGE_8G_FS, ADXL380_SENSITIVITY_8G},
		{ADXL380_RANGE_16G_FS, ADXL380_SENSITIVITY_16G},
	};

	run_range_cases(adxl380_dev, adxl380_emul, ranges, ARRAY_SIZE(ranges));
}

/* TC-DATA-002: ADXL382 uses a different sensitivity table and range codes. */
ZTEST(adxl38x_convert, test_accel_convert_adxl382)
{
	static const struct range_case ranges[] = {
		{ADXL382_RANGE_15G_FS, ADXL382_SENSITIVITY_15G},
		{ADXL382_RANGE_30G_FS, ADXL382_SENSITIVITY_30G},
		{ADXL382_RANGE_60G_FS, ADXL382_SENSITIVITY_60G},
	};

	run_range_cases(adxl382_dev, adxl382_emul, ranges, ARRAY_SIZE(ranges));
}

/* TC-DATA-002: the per-axis channel_get entries convert the right sample. */
ZTEST(adxl38x_convert, test_accel_single_axis_channels)
{
	struct sensor_value fs = {.val1 = ADXL380_RANGE_16G_FS, .val2 = 0};
	int16_t sens = ADXL380_SENSITIVITY_16G;
	struct sensor_value vx, vy, vz;

	zassert_ok(sensor_attr_set(adxl380_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &fs),
		   "FS set failed");

	/* X = +1 g, Y = -2 g, Z = +3 g. */
	seed_accel(adxl380_emul, sens, (int16_t)(-2 * sens), (int16_t)(3 * sens));
	zassert_ok(sensor_sample_fetch(adxl380_dev), "fetch failed");

	zassert_ok(sensor_channel_get(adxl380_dev, SENSOR_CHAN_ACCEL_X, &vx), "X get failed");
	zassert_ok(sensor_channel_get(adxl380_dev, SENSOR_CHAN_ACCEL_Y, &vy), "Y get failed");
	zassert_ok(sensor_channel_get(adxl380_dev, SENSOR_CHAN_ACCEL_Z, &vz), "Z get failed");

	check_ms2(&vx, (int64_t)SENSOR_G, "X(+1g)");
	check_ms2(&vy, -2 * (int64_t)SENSOR_G, "Y(-2g)");
	check_ms2(&vz, 3 * (int64_t)SENSOR_G, "Z(+3g)");
}

/* TC-DATA-003: die temperature. T(degC) = 25 + (raw - 550) / 10.2.
 * 550 -> 25.0, 652 (= 550 + 102) -> 35.0, 448 (= 550 - 102) -> 15.0.
 */
ZTEST(adxl38x_convert, test_temp_convert)
{
	struct sensor_value t;

	seed_temp(adxl380_emul, 550U);
	zassert_ok(sensor_sample_fetch(adxl380_dev), "fetch failed");
	zassert_ok(sensor_channel_get(adxl380_dev, SENSOR_CHAN_DIE_TEMP, &t), "temp get failed");
	zassert_equal(t.val1, 25, "25C: val1 %d != 25", t.val1);
	zassert_equal(t.val2, 0, "25C: val2 %d != 0", t.val2);

	seed_temp(adxl380_emul, 652U);
	zassert_ok(sensor_sample_fetch(adxl380_dev), "fetch failed");
	zassert_ok(sensor_channel_get(adxl380_dev, SENSOR_CHAN_DIE_TEMP, &t), "temp get failed");
	zassert_equal(t.val1, 35, "35C: val1 %d != 35", t.val1);
	zassert_equal(t.val2, 0, "35C: val2 %d != 0", t.val2);

	seed_temp(adxl380_emul, 448U);
	zassert_ok(sensor_sample_fetch(adxl380_dev), "fetch failed");
	zassert_ok(sensor_channel_get(adxl380_dev, SENSOR_CHAN_DIE_TEMP, &t), "temp get failed");
	zassert_equal(t.val1, 15, "15C: val1 %d != 15", t.val1);
	zassert_equal(t.val2, 0, "15C: val2 %d != 0", t.val2);
}

/* Restore the devicetree-default ranges so order-independent suites (the init
 * config-applied checks) still see the post-boot configuration.
 */
static void convert_teardown(void *fixture)
{
	struct sensor_value v = {.val2 = 0};

	ARG_UNUSED(fixture);
	v.val1 = ADXL380_RANGE_16G_FS;
	(void)sensor_attr_set(adxl380_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &v);
	v.val1 = ADXL382_RANGE_30G_FS;
	(void)sensor_attr_set(adxl382_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &v);
}

ZTEST_SUITE(adxl38x_convert, NULL, NULL, NULL, NULL, convert_teardown);
