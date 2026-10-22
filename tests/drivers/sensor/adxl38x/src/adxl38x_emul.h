/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Test-local register-file SPI emulator for the ADXL380/ADXL382 driver.
 * Exposes raw register peek/poke so tests can seed inputs and verify what the
 * driver wrote.
 */

#ifndef TESTS_DRIVERS_SENSOR_ADXL38X_EMUL_H_
#define TESTS_DRIVERS_SENSOR_ADXL38X_EMUL_H_

#include <stdint.h>
#include <zephyr/drivers/emul.h>

/** Return the current value of register @p reg in the emulator register file. */
uint8_t adxl38x_emul_get_reg(const struct emul *target, uint8_t reg);

/** Force register @p reg to @p val in the emulator register file. */
void adxl38x_emul_set_reg(const struct emul *target, uint8_t reg, uint8_t val);

#endif /* TESTS_DRIVERS_SENSOR_ADXL38X_EMUL_H_ */
