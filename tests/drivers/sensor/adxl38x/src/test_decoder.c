/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ADXL380/ADXL382 sensor-decoder tests (Track B, requirement group SRS-F-DEC).
 * Runs on native_sim and native_sim/native/64 against a hand-crafted FIFO buffer
 * (no live streaming needed — the decoder API is exercised directly).
 *
 * Covered here:
 *   TC-DEC-001  get_frame_count: FIFO count + chan_idx!=0 rejection (D2) (SRS-F-DEC-001)
 *   TC-DEC-002  stream decode + *fit continuation (BUG-001 gate)        (SRS-F-DEC-002)
 *
 * BUG-001: adxl38x_decode_stream() stores a pointer in the 32-bit *fit
 * (adxl38x_decoder.c). On a 64-bit host the pointer truncates, so the
 * continuation restarts at frame 0 every call (duplicate frame / never
 * completes). This suite asserts correct continuation, so it is EXPECTED TO
 * FAIL on native_sim/native/64 until BUG-001 is fixed, and to PASS on 32-bit
 * native_sim. The decode loop is bounded so the bug fails cleanly, never hangs.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

/* Private driver header: struct adxl38x_fifo_data layout, STATUS0 masks,
 * ADXL38X_SAMPLE_SIZE, chip/range enums, and adxl38x_get_decoder().
 */
#include "adxl38x.h"

#define ADXL380_NODE DT_NODELABEL(adxl380)

static const struct device *const dev = DEVICE_DT_GET(ADXL380_NODE);

#define N_SAMPLES       4
#define SAMPLE_SET_SIZE 6U /* X, Y, Z * ADXL38X_SAMPLE_SIZE */
#define TEST_ODR        8000U

static uint8_t fifo_buf[sizeof(struct adxl38x_fifo_data) + N_SAMPLES * SAMPLE_SET_SIZE];

/* Build a FIFO buffer: header + N three-axis sample sets with distinct values. */
static void build_fifo(void)
{
	struct adxl38x_fifo_data *hdr = (struct adxl38x_fifo_data *)fifo_buf;
	uint8_t *p = fifo_buf + sizeof(struct adxl38x_fifo_data);

	memset(fifo_buf, 0, sizeof(fifo_buf));
	hdr->is_fifo = 1;
	hdr->timestamp = 1000;
	hdr->status0 = ADXL38X_STATUS0_FIFO_WATERMARK_MSK;
	hdr->range = ADXL380_DT_RANGE_16G;
	hdr->chip_type = ADXL38X_CHIP_ADXL380;
	hdr->accel_odr = TEST_ODR;
	hdr->fifo_samples = N_SAMPLES;
	hdr->fifo_byte_count = N_SAMPLES * SAMPLE_SET_SIZE;
	hdr->sample_set_size = SAMPLE_SET_SIZE;

	for (int i = 0; i < N_SAMPLES; i++) {
		int16_t x = (int16_t)(1000 + i * 100);
		int16_t y = (int16_t)(2000 + i * 100);
		int16_t z = (int16_t)(3000 + i * 100);

		p[0] = (x >> 8) & 0xFF;
		p[1] = x & 0xFF;
		p[2] = (y >> 8) & 0xFF;
		p[3] = y & 0xFF;
		p[4] = (z >> 8) & 0xFF;
		p[5] = z & 0xFF;
		p += SAMPLE_SET_SIZE;
	}
}

static const struct sensor_decoder_api *get_decoder(void)
{
	const struct sensor_decoder_api *decoder = NULL;
	int ret = adxl38x_get_decoder(dev, &decoder);

	zassert_ok(ret, "adxl38x_get_decoder failed: %d", ret);
	zassert_not_null(decoder, "decoder is NULL");
	return decoder;
}

/* TC-DEC-001: FIFO frame count equals the stored sample count, and a non-zero
 * channel index is rejected so an iterating consumer terminates (D2 guard).
 */
ZTEST(adxl38x_decoder, test_frame_count)
{
	const struct sensor_decoder_api *decoder = get_decoder();
	struct sensor_chan_spec chan = {.chan_type = SENSOR_CHAN_ACCEL_XYZ, .chan_idx = 0};
	uint16_t fc = 0;

	build_fifo();

	zassert_ok(decoder->get_frame_count(fifo_buf, chan, &fc), "get_frame_count failed");
	zassert_equal(fc, N_SAMPLES, "frame count %u != %u", fc, N_SAMPLES);

	chan.chan_idx = 1;
	zassert_equal(decoder->get_frame_count(fifo_buf, chan, &fc), -ENOTSUP,
		      "chan_idx != 0 must be rejected (D2 regression)");
}

/* TC-DEC-002 (BUG-001 gate): decode one frame per call; a correct driver yields
 * N distinct frames with strictly increasing timestamp_delta, then returns 0.
 * The pointer-in-uint32_t *fit truncation duplicates frame 0 on 64-bit, which
 * the bounded loop + monotonicity check turn into a clean failure.
 */
ZTEST(adxl38x_decoder, test_stream_decode_continuation)
{
	const struct sensor_decoder_api *decoder = get_decoder();
	struct sensor_chan_spec chan = {.chan_type = SENSOR_CHAN_ACCEL_XYZ, .chan_idx = 0};
	struct sensor_three_axis_data out;
	uint32_t fit = 0;
	uint32_t last_delta = 0;
	int total = 0;

	build_fifo();

	for (int guard = 0; guard < N_SAMPLES + 8; guard++) {
		int c = decoder->decode(fifo_buf, chan, &fit, 1, &out);

		if (c == 0) {
			break;
		}
		zassert_equal(c, 1, "decode returned %d, expected 1 per call", c);

		uint32_t delta = out.readings[0].timestamp_delta;

		if (total > 0) {
			zassert_true(delta > last_delta,
				     "frame %d timestamp_delta %u not > previous %u "
				     "— duplicate frame (BUG-001 *fit truncation)",
				     total, delta, last_delta);
		}
		last_delta = delta;
		total++;
	}

	zassert_equal(total, N_SAMPLES,
		      "decoded %d frames, expected %d — *fit continuation broken "
		      "(BUG-001 pointer truncation on 64-bit)",
		      total, N_SAMPLES);
}

ZTEST_SUITE(adxl38x_decoder, NULL, NULL, NULL, NULL, NULL);
