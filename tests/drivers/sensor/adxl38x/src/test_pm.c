/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ADXL380/ADXL382 power-management test (Track B, requirement group SRS-F-PM).
 * Runs on native_sim and native_sim/native/64 against the register-file SPI
 * emulator. Requires CONFIG_PM_DEVICE=y.
 *
 * Covered here:
 *   TC-PM-001  PM_DEVICE suspend -> STANDBY, resume -> configured op-mode
 *              (SRS-F-PM-001)
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/pm/device.h>

#include "adxl38x.h"
#include "adxl38x_emul.h"

#define ADXL380_NODE DT_NODELABEL(adxl380)

static const struct device *const adxl380_dev = DEVICE_DT_GET(ADXL380_NODE);
static const struct emul *const adxl380_emul = EMUL_DT_GET(ADXL380_NODE);

/* TC-PM-001: suspend drives OP_MODE to standby; resume restores the devicetree
 * operating mode (HP for this overlay).
 */
ZTEST(adxl38x_pm, test_suspend_resume)
{
	zassert_ok(pm_device_action_run(adxl380_dev, PM_DEVICE_ACTION_SUSPEND), "suspend failed");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_OP_MODE) & ADXL38X_OP_MODE_MSK,
		      ADXL38X_STANDBY_MODE, "suspend did not enter standby");

	zassert_ok(pm_device_action_run(adxl380_dev, PM_DEVICE_ACTION_RESUME), "resume failed");
	zassert_equal(adxl38x_emul_get_reg(adxl380_emul, ADXL38X_OP_MODE) & ADXL38X_OP_MODE_MSK,
		      ADXL38X_HP_MODE, "resume did not restore the configured op-mode");
}

ZTEST_SUITE(adxl38x_pm, NULL, NULL, NULL, NULL, NULL);
