/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ADXL380/ADXL382 register-access tests (Track B, requirement group SRS-F-REG).
 * Runs on native_sim and native_sim/native/64 against the register-file SPI
 * emulator.
 *
 * Covered here:
 *   TC-REG-001  SPI register read/write/update round-trip (SRS-F-REG-001)
 *               - single-byte write then read-back
 *               - multi-byte burst (address auto-increment)
 *               - read-modify-write preserves the unmasked bits
 *               - same path on the ADXL382 instance
 *
 * The scratch registers below are arbitrary in-range addresses: the emulator is
 * a flat 128-byte file and these are not touched by init, so the raw R/W/update
 * path is exercised without any device-side effects. Each test seeds the value
 * it needs, so order between tests does not matter.
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>

/* Private driver header: adxl38x_reg_read/write/update() prototypes. */
#include "adxl38x.h"
#include "adxl38x_emul.h"

#define ADXL380_NODE DT_NODELABEL(adxl380)
#define ADXL382_NODE DT_NODELABEL(adxl382)

/* Scratch addresses, all < 0x80 and unused by the driver's init path. */
#define REG_SCRATCH    0x45U
#define REG_BURST_BASE 0x40U /* uses 0x40..0x43 */

static const struct device *const adxl380_dev = DEVICE_DT_GET(ADXL380_NODE);
static const struct device *const adxl382_dev = DEVICE_DT_GET(ADXL382_NODE);
static const struct emul *const adxl380_emul = EMUL_DT_GET(ADXL380_NODE);
static const struct emul *const adxl382_emul = EMUL_DT_GET(ADXL382_NODE);

/* TC-REG-001 (write/read): a written byte lands in the backend and reads back.
 * Two complementary patterns guard against a stuck bit passing by accident.
 */
ZTEST(adxl38x_regs, test_write_read_roundtrip)
{
	uint8_t wr;
	uint8_t rd = 0;

	wr = 0xA5U;
	zassert_ok(adxl38x_reg_write(adxl380_dev, REG_SCRATCH, &wr, 1), "write 0xA5 failed");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, REG_SCRATCH), 0xA5U,
		      "backend did not store 0xA5");
	zassert_ok(adxl38x_reg_read(adxl380_dev, REG_SCRATCH, &rd, 1), "read failed");
	zassert_equal(rd, 0xA5U, "read 0x%02x != 0xA5", rd);

	wr = 0x5AU;
	zassert_ok(adxl38x_reg_write(adxl380_dev, REG_SCRATCH, &wr, 1), "write 0x5A failed");
	zassert_ok(adxl38x_reg_read(adxl380_dev, REG_SCRATCH, &rd, 1), "read failed");
	zassert_equal(rd, 0x5AU, "read 0x%02x != 0x5A (stuck bit?)", rd);
}

/* TC-REG-001 (burst): a multi-byte write/read exercises the address
 * auto-increment in the driver's SPI framing and the emulator backend.
 */
ZTEST(adxl38x_regs, test_burst_roundtrip)
{
	uint8_t wr[4] = {0x11U, 0x22U, 0x33U, 0x44U};
	uint8_t rd[4] = {0};

	zassert_ok(adxl38x_reg_write(adxl380_dev, REG_BURST_BASE, wr, sizeof(wr)),
		   "burst write failed");

	for (size_t i = 0; i < sizeof(wr); i++) {
		uint8_t addr = (uint8_t)(REG_BURST_BASE + i);
		uint8_t got = adxl38x_emul_get_reg(adxl380_emul, addr);

		zassert_equal(got, wr[i], "backend reg[0x%02x]=0x%02x != 0x%02x", addr, got, wr[i]);
	}

	zassert_ok(adxl38x_reg_read(adxl380_dev, REG_BURST_BASE, rd, sizeof(rd)),
		   "burst read failed");
	zassert_mem_equal(rd, wr, sizeof(wr), "burst read-back mismatch");
}

/* TC-REG-001 (read-modify-write): update() rewrites only the masked field and
 * leaves the surrounding bits untouched. value is the unshifted field value;
 * FIELD_PREP aligns it into the mask (so 0x02 into mask 0xC0 becomes 0x80).
 */
ZTEST(adxl38x_regs, test_update_preserves_other_bits)
{
	uint8_t rd = 0;

	/* Low-nibble field: high nibble 0xA must survive. 0xA5 -> 0xAC. */
	adxl38x_emul_set_reg(adxl380_emul, REG_SCRATCH, 0xA5U);
	zassert_ok(adxl38x_reg_update(adxl380_dev, REG_SCRATCH, 0x0FU, 0x0CU), "RMW low failed");
	zassert_ok(adxl38x_reg_read(adxl380_dev, REG_SCRATCH, &rd, 1), "read failed");
	zassert_equal(rd, 0xACU, "RMW low nibble: 0x%02x != 0xAC", rd);

	/* Shifted field in bits 7:6: 0x3F with (2 << 6) -> 0xBF, low 6 bits kept. */
	adxl38x_emul_set_reg(adxl380_emul, REG_SCRATCH, 0x3FU);
	zassert_ok(adxl38x_reg_update(adxl380_dev, REG_SCRATCH, 0xC0U, 0x02U), "RMW high failed");
	zassert_ok(adxl38x_reg_read(adxl380_dev, REG_SCRATCH, &rd, 1), "read failed");
	zassert_equal(rd, 0xBFU, "RMW high field: 0x%02x != 0xBF (FIELD_PREP align?)", rd);
}

/* TC-REG-001 (second part): the same access path works on the ADXL382 instance
 * and its own emulator binding.
 */
ZTEST(adxl38x_regs, test_roundtrip_adxl382)
{
	uint8_t wr = 0x5AU;
	uint8_t rd = 0;

	zassert_ok(adxl38x_reg_write(adxl382_dev, REG_SCRATCH, &wr, 1), "ADXL382 write failed");
	zassert_ok(adxl38x_reg_read(adxl382_dev, REG_SCRATCH, &rd, 1), "ADXL382 read failed");
	zassert_equal(rd, 0x5AU, "ADXL382 read 0x%02x != 0x5A", rd);
	zassert_equal(adxl38x_emul_get_reg(adxl382_emul, REG_SCRATCH), 0x5AU,
		      "ADXL382 backend mismatch");
}

ZTEST_SUITE(adxl38x_regs, NULL, NULL, NULL, NULL, NULL);
