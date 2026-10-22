# Copyright (c) 2026 Analog Devices Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(adxl38x_test)

target_sources(app PRIVATE
	src/test_init.c
	src/test_decoder.c
	src/test_regs.c
	src/test_convert.c
	src/test_attr.c
	src/test_activity.c
	src/test_pm.c
	src/adxl38x_emul.c
)

# src/ for the emulator header; the driver dir for the private adxl38x.h
# (fifo-data layout + decoder prototype) used by the decoder test.
target_include_directories(app PRIVATE
	src
	${ZEPHYR_BASE}/drivers/sensor/adi/adxl38x
)
