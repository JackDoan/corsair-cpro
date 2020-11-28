// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * corsair-cpro.c - Linux driver for Corsair Commander Pro
 * Copyright (C) 2020 Marius Zachmann <mail@mariuszachmann.de>
 *
 * This driver uses hid reports to communicate with the device to allow hidraw userspace drivers
 * still being used. The device does not use report ids. When using hidraw and this driver
 * simultaniously, reports could be switched.
 */

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/hid.h>
#include <linux/hwmon.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>

#define USB_VENDOR_ID_CORSAIR			0x1b1c
#define USB_PRODUCT_ID_CORSAIR_COMMANDERPRO	0x0c10
#define USB_PRODUCT_ID_CORSAIR_1000D		0x1d00

#define OUT_BUFFER_SIZE		63
#define IN_BUFFER_SIZE		16
#define LABEL_LENGTH		11
#define REQ_TIMEOUT		300



#define NUM_FANS		6
#define NUM_TEMP_SENSORS	4

enum ccp_cmd {
    CTL_GET_TMP_CNCT=0x10, /* returns in bytes 1-4 for each temp sensor:, 0 not connected, 1 connected */

    CTL_GET_TMP=0x11,	/*
					 * send: byte 1 is channel, rest zero
					 * rcv:  returns temp for channel in centi-degree celsius
					 * in bytes 1 and 2
					 * returns 0x11 in byte 0 if no sensor is connected
					 */
    CTL_GET_VOLT=0x12,	/*
					 * send: byte 1 is rail number: 0 = 12v, 1 = 5v, 2 = 3.3v
					 * rcv:  returns millivolt in bytes 1,2
					 * returns error 0x10 if request is invalid
					 */
    CTL_GET_FAN_CNCT=0x20,	/*
					 * returns in bytes 1-6 for each fan:
					 * 0 not connected
					 * 1 3pin
					 * 2 4pin
					 */
    CTL_GET_FAN_RPM=0x21,	/*
					 * send: byte 1 is channel, rest zero
					 * rcv:  returns rpm in bytes 1,2
					 */
    CTL_GET_FAN_PWM=0x22,	/*
					 * send: byte 1 is channel, rest zero
					 * rcv:  returns pwm in byte 1 if it was set
					 *	 returns error 0x12 if fan is controlled via
					 *	 fan_target or fan curve
					 */
    CTL_SET_FAN_FPWM=0x23,	/*
					 * set fixed pwm
					 * send: byte 1 is fan number
					 * send: byte 2 is percentage from 0 - 100
					 */
    CTL_SET_FAN_TARGET=0x24,	/*
					 * set target rpm
					 * send: byte 1 is fan number
					 * send: byte 2-3 is target
					 * device accepts all values from 0x00 - 0xFFFF
					 */

    CTL_FAN_MODE_READ = 0x29, /* CMD 29 01 05 00, RESP 00 02 05 00 */
    //taken from OpenCorsairLink --device 0 --led channel=1,mode=4(rainbow),speed=1,strip_type=0x24(led per strip),strip_count=0
    CTL_LED_CONFIGURE_0x35 = 0x35,    /* CMD:
                             * 35 01 00 24 00 01 00 00 ff
                             * ff 00 00 ff 80 00 ff ff 00
                             * 00 23 00 2d 00 37 00 00 00
                             * RESP 00 */
    /*
    commands[0x00] = 0x35;
    commands[0x01] = ctrl->channel; // CLNP led_channel
    commands[0x02] = ctrl->strip_count * 10;
    commands[0x03] = ctrl->leds_per_strip; // 0x0A = LED Strip, 0x0C = HD Fan, 0x01 = SP Fan, 0x04 = ML Fan
    commands[0x04] = ctrl->mode;
    commands[0x05] = ctrl->speed;
    commands[0x06] = ctrl->direction;
    commands[0x07] = ctrl->color_change_style;
    commands[0x08] = 0xFF;
    commands[0x09] = ctrl->led_colors[0].red;
    commands[0x0A] = ctrl->led_colors[0].green;
    commands[0x0B] = ctrl->led_colors[0].blue;
    commands[0x0C] = ctrl->led_colors[1].red;
    commands[0x0D] = ctrl->led_colors[1].green;
    commands[0x0E] = ctrl->led_colors[1].blue;
    commands[0x0F] = ctrl->led_colors[2].red;
    commands[0x10] = ctrl->led_colors[2].green;
    commands[0x11] = ctrl->led_colors[2].blue;
    commands[0x12] = ( ctrl->temperatures[0] >> 8 );
    commands[0x13] = ( ctrl->temperatures[0] & 0xFF );
    commands[0x14] = ( ctrl->temperatures[1] >> 8 );
    commands[0x15] = ( ctrl->temperatures[1] & 0xFF );
    commands[0x16] = ( ctrl->temperatures[2] >> 8 );
    commands[0x17] = ( ctrl->temperatures[2] & 0xFF );
}*/
    CTL_LED_GROUP_RESET = 0x37, /* CMD 37 01 00, RESP 00 */
};


struct ccp_device {
	struct hid_device *hdev;
	struct device *hwmon_dev;
	struct completion wait_input_report;
	struct mutex mutex; /* whenever buffer is used, lock before send_usb_cmd */
	u8 *buffer;
	int target[6];
	DECLARE_BITMAP(temp_cnct, NUM_TEMP_SENSORS);
	DECLARE_BITMAP(fan_cnct, NUM_FANS);
	char fan_label[6][LABEL_LENGTH];
};

/* converts response error in buffer to errno */
static int ccp_get_errno(struct ccp_device *ccp)
{
	switch (ccp->buffer[0]) {
	case 0x00: /* success */
		return 0;
	case 0x01: /* called invalid command */
		return -EOPNOTSUPP;
	case 0x10: /* called GET_VOLT / GET_TMP with invalid arguments */
		return -EINVAL;
	case 0x11: /* requested temps of disconnected sensors */
	case 0x12: /* requested pwm of not pwm controlled channels */
		return -ENODATA;
	default:
		hid_dbg(ccp->hdev, "unknown device response error: %d", ccp->buffer[0]);
		return -EIO;
	}
}

/* send command, check for error in response, response in ccp->buffer */
static int send_usb_cmd_from_buf(struct ccp_device *ccp, u8* cmdbuf, int cmdbuflen)
{
    unsigned long t;
    int ret;
    size_t to_copy;
    to_copy = min(cmdbuflen, OUT_BUFFER_SIZE);

    memset(ccp->buffer, 0x00, OUT_BUFFER_SIZE);
    memcpy(ccp->buffer, cmdbuf, to_copy);

    reinit_completion(&ccp->wait_input_report);

    ret = hid_hw_output_report(ccp->hdev, ccp->buffer, OUT_BUFFER_SIZE);
    if (ret < 0)
        return ret;

    t = wait_for_completion_timeout(&ccp->wait_input_report, msecs_to_jiffies(REQ_TIMEOUT));
    if (!t)
        return -ETIMEDOUT;

    return ccp_get_errno(ccp);
}

/* send command, check for error in response, response in ccp->buffer */
static int send_usb_cmd(struct ccp_device *ccp, u8 command, u8 byte1, u8 byte2, u8 byte3)
{
    u8 buf[4];
    size_t buf_size = sizeof(buf);
	memset(buf, 0x00, buf_size);
	buf[0] = command;
	buf[1] = byte1;
	buf[2] = byte2;
	buf[3] = byte3;
	return send_usb_cmd_from_buf(ccp, buf, buf_size);
}



static int ccp_raw_event(struct hid_device *hdev, struct hid_report *report, u8 *data, int size)
{
	struct ccp_device *ccp = hid_get_drvdata(hdev);

	/* only copy buffer when requested */
	if (completion_done(&ccp->wait_input_report))
		return 0;

	memcpy(ccp->buffer, data, min(IN_BUFFER_SIZE, size));
	complete(&ccp->wait_input_report);

	return 0;
}

/* requests and returns single data values depending on channel */
static int get_data(struct ccp_device *ccp, int command, int channel, bool two_byte_data)
{
	int ret;

	mutex_lock(&ccp->mutex);

	ret = send_usb_cmd(ccp, command, channel, 0, 0);
	if (ret)
		goto out_unlock;

	ret = ccp->buffer[1];
	if (two_byte_data)
		ret = (ret << 8) + ccp->buffer[2];

out_unlock:
	mutex_unlock(&ccp->mutex);
	return ret;
}

static int set_pwm(struct ccp_device *ccp, int channel, long val)
{
	int ret;

	if (val < 0 || val > 255)
		return -EINVAL;

	/* The Corsair Commander Pro uses values from 0-100 */
	val = DIV_ROUND_CLOSEST(val * 100, 255);

	mutex_lock(&ccp->mutex);

	ret = send_usb_cmd(ccp, CTL_SET_FAN_FPWM, channel, val, 0);
	if (!ret)
		ccp->target[channel] = -ENODATA;

	mutex_unlock(&ccp->mutex);
	return ret;
}

static int set_target(struct ccp_device *ccp, int channel, long val)
{
	int ret;

	val = clamp_val(val, 0, 0xFFFF);
	ccp->target[channel] = val;

	mutex_lock(&ccp->mutex);
	ret = send_usb_cmd(ccp, CTL_SET_FAN_TARGET, channel, val >> 8, val);

	mutex_unlock(&ccp->mutex);
	return ret;
}

static int ccp_read_string(struct device *dev, enum hwmon_sensor_types type,
			   u32 attr, int channel, const char **str)
{
	struct ccp_device *ccp = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_label:
			*str = ccp->fan_label[channel];
			return 0;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static int ccp_read(struct device *dev, enum hwmon_sensor_types type,
		    u32 attr, int channel, long *val)
{
	struct ccp_device *ccp = dev_get_drvdata(dev);
	int ret;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			ret = get_data(ccp, CTL_GET_TMP, channel, true);
			if (ret < 0)
				return ret;
			*val = ret * 10;
			return 0;
		default:
			break;
		}
		break;
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_input:
			ret = get_data(ccp, CTL_GET_FAN_RPM, channel, true);
			if (ret < 0)
				return ret;
			*val = ret;
			return 0;
		case hwmon_fan_target:
			/* how to read target values from the device is unknown */
			/* driver returns last set value or 0			*/
			if (ccp->target[channel] < 0)
				return -ENODATA;
			*val = ccp->target[channel];
			return 0;
		default:
			break;
		}
		break;
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			ret = get_data(ccp, CTL_GET_FAN_PWM, channel, false);
			if (ret < 0)
				return ret;
			*val = DIV_ROUND_CLOSEST(ret * 255, 100);
			return 0;
		default:
			break;
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			ret = get_data(ccp, CTL_GET_VOLT, channel, true);
			if (ret < 0)
				return ret;
			*val = ret;
			return 0;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return -EOPNOTSUPP;
};

static int ccp_write(struct device *dev, enum hwmon_sensor_types type,
		     u32 attr, int channel, long val)
{
	struct ccp_device *ccp = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			return set_pwm(ccp, channel, val);
		default:
			break;
		}
		break;
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_target:
			return set_target(ccp, channel, val);
		default:
			break;
		}
	default:
		break;
	}

	return -EOPNOTSUPP;
};

static umode_t ccp_is_visible(const void *data, enum hwmon_sensor_types type,
			      u32 attr, int channel)
{
	const struct ccp_device *ccp = data;

	switch (type) {
	case hwmon_temp:
		if (!test_bit(channel, ccp->temp_cnct))
			break;

		switch (attr) {
		case hwmon_temp_input:
			return 0444;
		case hwmon_temp_label:
			return 0444;
		default:
			break;
		}
		break;
	case hwmon_fan:
		if (!test_bit(channel, ccp->fan_cnct))
			break;

		switch (attr) {
		case hwmon_fan_input:
			return 0444;
		case hwmon_fan_label:
			return 0444;
		case hwmon_fan_target:
			return 0644;
		default:
			break;
		}
		break;
	case hwmon_pwm:
		if (!test_bit(channel, ccp->fan_cnct))
			break;

		switch (attr) {
		case hwmon_pwm_input:
			return 0644;
		default:
			break;
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			return 0444;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return 0;
};

static const struct hwmon_ops ccp_hwmon_ops = {
	.is_visible = ccp_is_visible,
	.read = ccp_read,
	.read_string = ccp_read_string,
	.write = ccp_write,
};

static const struct hwmon_channel_info *ccp_info[] = {
	HWMON_CHANNEL_INFO(chip,
			   HWMON_C_REGISTER_TZ),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT,
			   HWMON_T_INPUT,
			   HWMON_T_INPUT,
			   HWMON_T_INPUT
			   ),
	HWMON_CHANNEL_INFO(fan,
			   HWMON_F_INPUT | HWMON_F_LABEL | HWMON_F_TARGET,
			   HWMON_F_INPUT | HWMON_F_LABEL | HWMON_F_TARGET,
			   HWMON_F_INPUT | HWMON_F_LABEL | HWMON_F_TARGET,
			   HWMON_F_INPUT | HWMON_F_LABEL | HWMON_F_TARGET,
			   HWMON_F_INPUT | HWMON_F_LABEL | HWMON_F_TARGET,
			   HWMON_F_INPUT | HWMON_F_LABEL | HWMON_F_TARGET
			   ),
	HWMON_CHANNEL_INFO(pwm,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT
			   ),
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT,
			   HWMON_I_INPUT,
			   HWMON_I_INPUT
			   ),
	NULL
};

static const struct hwmon_chip_info ccp_chip_info = {
	.ops = &ccp_hwmon_ops,
	.info = ccp_info,
};

/* read fan connection status and set labels */
static int get_fan_cnct(struct ccp_device *ccp)
{
	int channel;
	int mode;
	int ret;

	ret = send_usb_cmd(ccp, CTL_GET_FAN_CNCT, 0, 0, 0);
	if (ret)
		return ret;

	for (channel = 0; channel < NUM_FANS; channel++) {
		mode = ccp->buffer[channel + 1];
		if (mode == 0)
			continue;

		set_bit(channel, ccp->fan_cnct);
		ccp->target[channel] = -ENODATA;

		switch (mode) {
		case 1:
			scnprintf(ccp->fan_label[channel], LABEL_LENGTH,
				  "fan%d 3pin", channel + 1);
			break;
		case 2:
			scnprintf(ccp->fan_label[channel], LABEL_LENGTH,
				  "fan%d 4pin", channel + 1);
			break;
		default:
			scnprintf(ccp->fan_label[channel], LABEL_LENGTH,
				  "fan%d other", channel + 1);
			break;
		}
	}

	return 0;
}

/* read temp sensor connection status */
static int get_temp_cnct(struct ccp_device *ccp)
{
	int channel;
	int mode;
	int ret;

	ret = send_usb_cmd(ccp, CTL_GET_TMP_CNCT, 0, 0, 0);
	if (ret)
		return ret;

	for (channel = 0; channel < NUM_TEMP_SENSORS; channel++) {
		mode = ccp->buffer[channel + 1];
		if (mode == 0)
			continue;

		set_bit(channel, ccp->temp_cnct);
	}

	return 0;
}

static int ccp_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct ccp_device *ccp;
	int ret;

	ccp = devm_kzalloc(&hdev->dev, sizeof(*ccp), GFP_KERNEL);
	if (!ccp)
		return -ENOMEM;

	ccp->buffer = devm_kmalloc(&hdev->dev, OUT_BUFFER_SIZE, GFP_KERNEL);
	if (!ccp->buffer)
		return -ENOMEM;

	ret = hid_parse(hdev);
	if (ret)
		return ret;

	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret)
		return ret;

	ret = hid_hw_open(hdev);
	if (ret)
		goto out_hw_stop;

	ccp->hdev = hdev;
	hid_set_drvdata(hdev, ccp);
	mutex_init(&ccp->mutex);
	init_completion(&ccp->wait_input_report);

	hid_device_io_start(hdev);

	/* temp and fan connection status only updates when device is powered on */
	ret = get_temp_cnct(ccp);
	if (ret)
		goto out_hw_close;

	ret = get_fan_cnct(ccp);
	if (ret)
		goto out_hw_close;
	ccp->hwmon_dev = hwmon_device_register_with_info(&hdev->dev, "corsaircpro",
							 ccp, &ccp_chip_info, 0);
	if (IS_ERR(ccp->hwmon_dev)) {
		ret = PTR_ERR(ccp->hwmon_dev);
		goto out_hw_close;
	}

	return 0;

out_hw_close:
	hid_hw_close(hdev);
out_hw_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void ccp_remove(struct hid_device *hdev)
{
	struct ccp_device *ccp = hid_get_drvdata(hdev);

	hwmon_device_unregister(ccp->hwmon_dev);
	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static const struct hid_device_id ccp_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_CORSAIR, USB_PRODUCT_ID_CORSAIR_COMMANDERPRO) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_CORSAIR, USB_PRODUCT_ID_CORSAIR_1000D) },
	{ }
};

static struct hid_driver ccp_driver = {
	.name = "corsair-cpro",
	.id_table = ccp_devices,
	.probe = ccp_probe,
	.remove = ccp_remove,
	.raw_event = ccp_raw_event,
};

MODULE_DEVICE_TABLE(hid, ccp_devices);
MODULE_LICENSE("GPL");

static int __init ccp_init(void)
{
	return hid_register_driver(&ccp_driver);
}

static void __exit ccp_exit(void)
{
	hid_unregister_driver(&ccp_driver);
}

/*
 * When compiling this driver as built-in, hwmon initcalls will get called before the
 * hid driver and this driver would fail to register. late_initcall solves this.
 */
late_initcall(ccp_init);
module_exit(ccp_exit);
