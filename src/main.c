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
#include <zephyr/drivers/sensor.h>
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
#define ENCODER_COUNTS_PER_REV DT_PROP(DT_ALIAS(qdec0), st_counts_per_revolution)

const struct device *const dev_l = DEVICE_DT_GET(DT_ALIAS(qdec0));
const struct device *const dev_r = DEVICE_DT_GET(DT_ALIAS(qdec2));
struct sensor_value val;
struct sensor_value val2;

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

	HID_USAGE_PAGE(HID_USAGE_GEN_DESKTOP),
	HID_USAGE(HID_USAGE_GEN_HATSWITCH),
	HID_LOGICAL_MIN8(0),
	HID_LOGICAL_MAX8(7),
	// Physical Maximum  : 315 degrees (Optional)
	0x46, 0x3B, 0x01,
	HID_REPORT_SIZE(8),
	HID_REPORT_COUNT(1),
	// Unit: English Rotation/Angular Position 1 degree (Optional)
	0x65, 0x14,
	// Input: Data, Var, Abs, Null State
	HID_INPUT(0x42),
	HID_END_COLLECTION};
static enum usb_dc_status_code usb_status;

enum box_report_idx
{
	BUTTONS_G0_IDX = 0,
	BUTTONS_G1_IDX = 1,
	HAT_IDX = 2,

	BUTTONS_DOWN_IDX = 0,
	BUTTONS_UP_IDX = 1,
	HAT_CLICK_IDX = 2,
	BUTTON_0_IDX = 3,
	BUTTON_1_IDX = 4,

};

static uint8_t report[REPORT_SIZE];
static uint32_t report_buttons;
static uint32_t hat_bits;
static int loop_counter = 2;
static int clonks_to_clonk = 0;
static int angle_prev;
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
		WRITE_BIT(hat_bits, HAT_BIT_UP, evt->value);

		break;
	case HAT_BIT_LEFT:
		rwup_if_suspended();
		WRITE_BIT(hat_bits, HAT_BIT_LEFT, evt->value);
		break;
	case HAT_BIT_RIGHT:
		rwup_if_suspended();
		WRITE_BIT(hat_bits, HAT_BIT_RIGHT, evt->value);
		break;
	case HAT_BIT_DOWN:
		rwup_if_suspended();
		WRITE_BIT(hat_bits, HAT_BIT_DOWN, evt->value);
		break;
	case HAT_BIT_CLICK:
		rwup_if_suspended();
		WRITE_BIT(hat_bits, HAT_BIT_CLICK, evt->value);
		break;

	case BUTTON_0:
		rwup_if_suspended();
		WRITE_BIT(report_buttons, BUTTON_0_IDX, evt->value);
		break;
	case BUTTON_1:
		rwup_if_suspended();
		WRITE_BIT(report_buttons, BUTTON_1_IDX, evt->value);
		break;

	default:
		return;
	}
	k_mutex_unlock(&test_mutex);
}

/// @brief Does a positive modulo operation so the stack exchange answer works
/// @param a Input
/// @param n Modulus
/// @return
int mod_positive(int a, int n)
{
	return (a % n + n) % n;
}

static void read_encoder()
{
	// do all the encoder memes

	// Up button
	if (clonks_to_clonk > 0)
	{
		WRITE_BIT(report_buttons, BUTTONS_DOWN_IDX, 0);

		if (loop_counter > 0)
		{
			// Push button
			WRITE_BIT(report_buttons, BUTTONS_UP_IDX, 1);
			loop_counter--;
		}
		else
		{
			// Release button
			WRITE_BIT(report_buttons, BUTTONS_UP_IDX, 0);
			loop_counter--;
			if (loop_counter == -1)
			{
				printk("LED set 0");
				clonks_to_clonk--;
				loop_counter = 2;
			}
		}
	}

	// Down button
	if (clonks_to_clonk < 0)
	{
		WRITE_BIT(report_buttons, BUTTONS_UP_IDX, 0);

		if (loop_counter > 0)
		{
			// Push button
			WRITE_BIT(report_buttons, BUTTONS_DOWN_IDX, 1);

			loop_counter--;
		}
		else
		{
			// Release button
			WRITE_BIT(report_buttons, BUTTONS_DOWN_IDX, 0);
			loop_counter--;
			if (loop_counter == -1)
			{
				clonks_to_clonk++;
				loop_counter = 2;
			}
		}
	}

	if (clonks_to_clonk == 0)
	{
		// reset
		loop_counter = 2;
		// release buttons
		WRITE_BIT(report_buttons, BUTTONS_DOWN_IDX, 0);
		WRITE_BIT(report_buttons, BUTTONS_UP_IDX, 0);
	}
	int rc;
	rc = sensor_sample_fetch(dev_l);
	if (rc != 0)
	{
		printk("Failed to fetch sample (%d)\n", rc);
		return 0;
	}
	rc = sensor_sample_fetch(dev_r);
	if (rc != 0)
	{
		printk("Failed to fetch sample (%d)\n", rc);
		return 0;
	}

	rc = sensor_channel_get(dev_l, SENSOR_CHAN_ROTATION, &val);
	if (rc != 0)
	{
		printk("Failed to get data (%d)\n", rc);
		return 0;
	}

	rc = sensor_sample_fetch(dev_l);
	if (rc != 0)
	{
		printk("Failed to fetch sample (%d)\n", rc);
		return 0;
	}

	rc = sensor_channel_get(dev_r, SENSOR_CHAN_ROTATION, &val2);
	if (rc != 0)
	{
		printk("Failed to get data (%d)\n", rc);
		return 0;
	}

	int a = val.val1 - angle_prev;

	a = mod_positive((a + 180), 360) - 180;

	angle_prev = val.val1;

	// printk("Delta = %d degrees\n", a);

	int clonks = a / (360 / ENCODER_COUNTS_PER_REV);
	clonks_to_clonk += clonks;
	printk("Clonks = %d clonks\n", clonks);
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

	if (hat_bits == 1 << HAT_BIT_CLICK)
	{
		WRITE_BIT(report_buttons, HAT_CLICK_IDX, 1);
	}
	else
	{
		WRITE_BIT(report_buttons, HAT_CLICK_IDX, 0);
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
		read_encoder();
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
