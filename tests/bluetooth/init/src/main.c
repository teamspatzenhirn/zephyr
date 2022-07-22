/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <ztest.h>

#include <zephyr/bluetooth/bluetooth.h>

ZTEST_SUITE(test_bluetooth_init, NULL, NULL, NULL, NULL, NULL);

ZTEST(test_bluetooth_init, test_init)
{
	zassert_false(bt_enable(NULL), "Bluetooth init failed");
}
