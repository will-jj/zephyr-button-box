/*
 * Copyright (c) 2018 qianfan Zhao
 * Copyright (c) 2018, 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/util.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include "button_defs.h"
#include <zephyr/logging/log.h>

#define REPORT_SIZE 3
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

K_MUTEX_DEFINE(test_mutex);

#define THREAD_REDLINE_PRIORITY 5
#define REDLINE_STACK_SIZE 500

K_THREAD_STACK_DEFINE(redline_stack_area, REDLINE_STACK_SIZE);
struct k_thread redline_thread_data;
#define REDLINE_DELAY_TIME K_MSEC(50)

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const uint8_t hid_report_desc[] = {

	HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
	HID_USAGE(HID_USAGE_GEN_DESKTOP_GAMEPAD),
	HID_COLLECTION(HID_COLLECTION_APPLICATION),
	HID_USAGE_PAGE(HID_USAGE_GEN_BUTTON),
	HID_USAGE_MIN8(1),
	HID_USAGE_MAX8(16),
	HID_LOGICAL_MIN8(0),
	HID_LOGICAL_MAX8(1),
	HID_REPORT_SIZE(1),
	HID_REPORT_COUNT(16),
	HID_INPUT(0x02),

	0x05, 0x01,		  // Usage Page        : Generic Desktop
	0x09, 0x39,		  // Usage             : Hat Switch,
	0x15, 0x00,		  // Logical Min       : 0
	0x25, 0x07,		  // Logical Max       : 7
	0x46, 0x3B, 0x01, // Physical Maximum  : 315 degrees (Optional)
	0x75, 0x08,		  // Report Size       : 8
	0x95, 0x01,		  // Report Count      : 1
	0x65, 0x14,		  // Unit              : English Rotation/Angular Position 1 degree (Optional)
	0x81, 0x42,		  // Input             : Data, Var, Abs, Null State

	HID_END_COLLECTION};
static enum usb_dc_status_code usb_status;

enum box_report_idx
{
	BUTTONS_G0_IDX = 0,
	BUTTONS_G1_IDX = 1,
	HAT_IDX = 2,
};

static uint8_t report[REPORT_SIZE];
static uint32_t report_buttons;
static uint32_t hat_bits;

static K_SEM_DEFINE(report_sem, 0, 1);

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	usb_status = status;
}

static ALWAYS_INLINE void rwup_if_suspended(void)
{
	if (IS_ENABLED(CONFIG_USB_DEVICE_REMOTE_WAKEUP))
	{
		if (usb_status == USB_DC_SUSPEND)
		{
			usb_wakeup_request();
			return;
		}
	}
}

static void input_cb(struct input_event *evt)
{
	k_mutex_lock(&test_mutex, K_FOREVER);
	LOG_INF("Input code %u value %d",
			evt->code, evt->value);
	switch (evt->code)
	{
	case HAT_BIT_UP:
		rwup_if_suspended();
		WRITE_BIT(report_buttons, HAT_BIT_UP, evt->value);
		WRITE_BIT(hat_bits, HAT_BIT_UP, evt->value);

		break;
	case HAT_BIT_LEFT:
		rwup_if_suspended();
		WRITE_BIT(report_buttons, HAT_BIT_LEFT, evt->value);
		WRITE_BIT(hat_bits, HAT_BIT_LEFT, evt->value);
		break;
	case HAT_BIT_RIGHT:
		rwup_if_suspended();
		WRITE_BIT(report_buttons, HAT_BIT_RIGHT, evt->value);
		WRITE_BIT(hat_bits, HAT_BIT_RIGHT, evt->value);
		break;
	case HAT_BIT_DOWN:
		rwup_if_suspended();
		WRITE_BIT(report_buttons, HAT_BIT_DOWN, evt->value);
		WRITE_BIT(hat_bits, HAT_BIT_DOWN, evt->value);
		break;
	case HAT_BIT_CLICK:
		rwup_if_suspended();
		WRITE_BIT(report_buttons, HAT_BIT_CLICK, evt->value);
		WRITE_BIT(hat_bits, HAT_BIT_CLICK, evt->value);
		break;

	default:
		return;
	}
	k_mutex_unlock(&test_mutex);
}

static uint8_t get_hat()
{
	uint8_t hat;

	// LOG_INF("Entering hat");
	// LOG_INF("Hat Bits : %d", hat_bits);
	if (hat_bits & 1 << HAT_BIT_CLICK)
	{

		LOG_INF("Centre hat clicked");
		switch (hat_bits & ~(1 << HAT_BIT_CLICK))
		{
		case 1 << HAT_BIT_UP:
			LOG_INF("UP");

			hat = HATSWITCH_UP;
			break;
		case 1 << HAT_BIT_DOWN:
			hat = HATSWITCH_DOWN;
			break;
		case 1 << HAT_BIT_LEFT:
			hat = HATSWITCH_LEFT;
			break;
		case 1 << HAT_BIT_RIGHT:
			hat = HATSWITCH_RIGHT;
			break;
		case 1 << HAT_BIT_UP | 1 << HAT_BIT_LEFT:
			hat = HATSWITCH_UPLEFT;
			break;
		case 1 << HAT_BIT_DOWN | 1 << HAT_BIT_LEFT:
			hat = HATSWITCH_DOWNLEFT;
			break;
		case 1 << HAT_BIT_UP | 1 << HAT_BIT_RIGHT:
			hat = HATSWITCH_UPRIGHT;
			break;
		case 1 << HAT_BIT_DOWN | 1 << HAT_BIT_RIGHT:
			hat = HATSWITCH_DOWNRIGHT;
			break;
		default:
			hat = HATSWITCH_NONE;
			break;
		}
	}
	else
	{
		hat = HATSWITCH_NONE;
	}
	return hat;
}

static void send_report(void)
{
	uint8_t tmp[REPORT_SIZE];
	(void)memcpy(tmp, report, sizeof(tmp));

	tmp[BUTTONS_G0_IDX] = (report_buttons & 0xFF);
	tmp[BUTTONS_G1_IDX] = (report_buttons >> 8);

	tmp[HAT_IDX] = get_hat();

	if (memcmp(tmp, report, sizeof(tmp)))
	{
		memcpy(report, tmp, sizeof(report));
		k_sem_give(&report_sem);
		LOG_INF("HII");
	}
}

INPUT_CALLBACK_DEFINE(NULL, input_cb);

void thread_red_line(void)
{
	while (1)
	{
		send_report();
		k_sleep(REDLINE_DELAY_TIME);
	}
}

int main(void)
{
	const struct device *hid_dev;
	int ret;

	if (!gpio_is_ready_dt(&led0))
	{
		LOG_ERR("LED device %s is not ready", led0.port->name);
		return 0;
	}

	hid_dev = device_get_binding("HID_0");
	if (hid_dev == NULL)
	{
		LOG_ERR("Cannot get USB HID Device");
		return 0;
	}

	ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT);
	if (ret < 0)
	{
		LOG_ERR("Failed to configure the LED pin, error: %d", ret);
		return 0;
	}

	usb_hid_register_device(hid_dev,
							hid_report_desc, sizeof(hid_report_desc),
							NULL);

	usb_hid_init(hid_dev);

	ret = usb_enable(status_cb);
	if (ret != 0)
	{
		LOG_ERR("Failed to enable USB");
		return 0;
	}

	k_tid_t my_tid = k_thread_create(&redline_thread_data, redline_stack_area,
									 K_THREAD_STACK_SIZEOF(redline_stack_area),
									 thread_red_line,
									 NULL, NULL, NULL,
									 THREAD_REDLINE_PRIORITY, 0, K_NO_WAIT);

	while (true)
	{
		//

		k_sem_take(&report_sem, K_FOREVER);
		ret = hid_int_ep_write(hid_dev, report, sizeof(report), NULL);

		if (ret)
		{
			LOG_ERR("HID write error, %d", ret);
		}

		/* Toggle LED on sent report */
		ret = gpio_pin_toggle(led0.port, led0.pin);
		if (ret < 0)
		{
			LOG_ERR("Failed to toggle the LED pin, error: %d", ret);
		}
		k_sleep(K_MSEC(20));
	}
	return 0;
}
