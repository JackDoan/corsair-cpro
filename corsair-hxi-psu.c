// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * corsair-cpro.c - Linux driver for Corsair HX 850i
 * Copyright (C) 2020 Jack Doan <me@jackdoan.com>
 * Copyright (C) 2020 Marius Zachmann <mail@mariuszachmann.de>
 * Copyright (C) 2017-2019  Sean Nelson <audiohacked@gmail.com>
 *
 * This driver uses hid reports to communicate with the device to allow hidraw userspace drivers
 * still being used. The device does not use report ids. When using hidraw and this driver
 * simultaneously, reports could be switched.
 */
/*
 * Reference:
 * Checking USB device 17 (1b1c:1c06)...
Corsair product detected. Checking if corsair_device is HX850i... Dev=1, CorsairLink Device Found: HX850i!
DEBUG: scan done, start routines
DEBUG: selected device_number = 1
DEBUG: shortcuts set
DEBUG: init done
Vendor: CORSAIR
Product: HX850i
Firmware: NA
DEBUG: string done
03 8D C3 F0 00 00
Temperature 0: 50.16 C
03 8E 9C F0 00 00
Temperature 1: 40.18 C
Powered: 33776986 (390d.  22h)
Uptime: 8929786 (103d.  8h)
DEBUG: time done
03 88 E6 F8 00 00
Supply Voltage: 115.00 V
03 EE 6D 08 00 00
Total Watts: 220.00 W
DEBUG: supply done
Output 12v:
03 8B 09 D3 00 00
	Voltage 12.16 V
03 8C 3D F0 00 00
	Amps 15.50 A
03 96 5C 08 00 00
	Watts 184.00 W
Output 5v:
03 8B 40 D1 00 00
	Voltage  5.00 V
03 8C 43 E0 00 00
	Amps  4.25 A
03 96 29 F8 00 00
	Watts 21.00 W
Output 3.3v:
03 8B D2 D0 00 00
	Voltage  3.28 V
03 8C 35 E0 00 00
	Amps  3.38 A
03 96 15 F8 00 00
	Watts 11.00 W
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
#define USB_PRODUCT_ID_CORSAIR_HX850i	0x1c06

#define OUT_BUFFER_SIZE		63
#define IN_BUFFER_SIZE		16
#define LABEL_LENGTH		11
#define REQ_TIMEOUT		300

#define NUM_FANS 1
#define NUM_TEMP_SENSORS 2

struct hxi_device {
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
static int hxi_get_errno(struct hxi_device *hxi)
{
    //todo
    return 0;
//	switch (hxi->buffer[0]) {
//	case 0x00: /* success */
//		return 0;
//	case 0x01: /* called invalid command */
//		return -EOPNOTSUPP;
//	case 0x10: /* called GET_VOLT / GET_TMP with invalid arguments */
//		return -EINVAL;
//	case 0x11: /* requested temps of disconnected sensors */
//	case 0x12: /* requested pwm of not pwm controlled channels */
//		return -ENODATA;
//	default:
//		hid_dbg(hxi->hdev, "unknown device response error: %d", hxi->buffer[0]);
//		return -EIO;
//	}
}

/* send command, check for error in response, response in hxi->buffer */
static int send_usb_cmd(struct hxi_device *hxi, u8 command, u8 byte1, u8 byte2, u8 byte3)
{
	unsigned long t;
	int ret;

	memset(hxi->buffer, 0x00, OUT_BUFFER_SIZE);
	hxi->buffer[0] = command;
	hxi->buffer[1] = byte1;
	hxi->buffer[2] = byte2;
	hxi->buffer[3] = byte3;

	reinit_completion(&hxi->wait_input_report);

	ret = hid_hw_output_report(hxi->hdev, hxi->buffer, OUT_BUFFER_SIZE);
	if (ret < 0) {
        pr_info("output report response error: %d", ret);
        return ret;
    }
    pr_info("output report response got bytes: %d", ret);
	t = wait_for_completion_timeout(&hxi->wait_input_report, msecs_to_jiffies(REQ_TIMEOUT));
	if (!t) {
        pr_info("timeout", ret);
        return -ETIMEDOUT;
    }

	return hxi_get_errno(hxi);
}

static int hxi_raw_event(struct hid_device *hdev, struct hid_report *report, u8 *data, int size)
{
	struct hxi_device *hxi = hid_get_drvdata(hdev);

	/* only copy buffer when requested */
	if (completion_done(&hxi->wait_input_report))
		return 0;

	memcpy(hxi->buffer, data, min(IN_BUFFER_SIZE, size));
	complete(&hxi->wait_input_report);

	return 0;
}


static int get_temperature(struct hxi_device *hxi, int channel) {
    int ret;
    const u8 get_temp_cmd = 0x8D;
    const u8 len = 0x03;
    u8 cmd = get_temp_cmd + min((u8)channel, (u8)1); //todo: find idiom for handling bad channel ID
    mutex_lock(&hxi->mutex);

    ret = send_usb_cmd(hxi, len, cmd, 0, 0);
    if (ret) { //failure
        pr_info("bad getTemp: %x", ret);
        ret = -ENODATA;
    }
    else { //success
        ret = (hxi->buffer[2] << 8) + hxi->buffer[3];
    }
    mutex_unlock(&hxi->mutex);
    return ret;
}

enum hxi_sensor_id {
    SENSOR_12V = 0x0,
    SENSOR_5V  = 0x1,
    SENSOR_3V3 = 0x2,
    SENSOR_TOTAL = 0xFE // do not send
};

enum hxi_sensor_signal {
    SIG_VOLTS = 0x8B,
    SIG_WALL_VOLTS = 0x88,
    SIG_AMPS  = 0x8C,
    SIG_WATTS = 0x96,
    SIG_TOTAL_WATTS = 0xEE,
};

static int decode_corsair_float(uint16_t v16 )
{
    int ret;
    int exponent = v16 >> 11;
    int fraction = (int)( v16 & 2047 );
    if ( exponent > 15 )
        exponent = -( 32 - exponent );

    if ( fraction > 1023 )
        fraction = -( 2048 - fraction );

    if ( ( fraction & 1 ) == 1 )
        fraction++;

    fraction = fraction * 1000; // fix unit scaling, maintain precision before shifts
    if( exponent < 0) {
        ret = fraction >> ((~exponent)+1);
    }
    else {
        ret = fraction << exponent;
    }
    return ret;
}

/* requests and returns single data values depending on channel */
static int get_electric_data(struct hxi_device *hxi, enum hxi_sensor_id sensor, enum hxi_sensor_signal sig)
{
	int ret;
	mutex_lock(&hxi->mutex);
    switch (sensor) {
        case SENSOR_12V:
        case SENSOR_5V:
        case SENSOR_3V3:
            ret = send_usb_cmd(hxi, 0x02, 0x0, sensor, 0);
            break;
        case SENSOR_TOTAL:
            ret = 0;
            break;
        default:
            ret = -1;
            break;
    }
    if (ret) {
        pr_info("bad error1: %x", ret);
        goto out_unlock;
    }
    switch (sig) {
        case SIG_VOLTS:
        case SIG_AMPS:
        case SIG_WATTS:
        case SIG_TOTAL_WATTS:
        case SIG_WALL_VOLTS:
            ret = send_usb_cmd(hxi, 0x3, sig, 0, 0);
            break;
        default:
            ret = -1;
            break;
    }
    if (ret) {
        pr_info("bad error2: %x", ret);
        goto out_unlock;
    }
    //this is different byte order from temperature. Corsair, why?
    ret = (hxi->buffer[3] << 8) + hxi->buffer[2];

out_unlock:
	mutex_unlock(&hxi->mutex);
	return decode_corsair_float(ret);
}

static int hxi_read_string(struct device *dev, enum hwmon_sensor_types type,
			   u32 attr, int channel, const char **str)
{
	struct hxi_device *hxi = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_label:
			*str = hxi->fan_label[channel];
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

static int hxi_read(struct device *dev, enum hwmon_sensor_types type,
		    u32 attr, int channel, long *val)
{
	struct hxi_device *hxi = dev_get_drvdata(dev);
	int ret;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			ret = get_temperature(hxi, channel);
			if (ret < 0) {
                return ret;
            }
			*val = ret;
			return 0;
		default:
			break;
		}
		break;
    case hwmon_in:
        switch (attr) {
            case hwmon_in_input:
                switch(channel) {
                    case 0:
                        ret = get_electric_data(hxi, SENSOR_12V, SIG_VOLTS);
                        break;
                    case 1:
                        ret = get_electric_data(hxi, SENSOR_5V, SIG_VOLTS);
                        break;
                    case 2:
                        ret = get_electric_data(hxi, SENSOR_3V3, SIG_VOLTS);
                        break;
                    case 3:
                        ret = get_electric_data(hxi, SENSOR_TOTAL, SIG_WALL_VOLTS);
                        break;
                }
                if (ret < 0) {
                    return ret;
                }
                *val = ret;
                return 0;
            default:
                break;
        }
        break;
    case hwmon_curr:
        switch (attr) {
            case hwmon_in_input:
                switch(channel) {
                    case 0:
                        ret = get_electric_data(hxi, SENSOR_12V, SIG_AMPS);
                        break;
                    case 1:
                        ret = get_electric_data(hxi, SENSOR_5V, SIG_AMPS);
                        break;
                    case 2:
                        ret = get_electric_data(hxi, SENSOR_3V3, SIG_AMPS);
                        break;
                }
                if (ret < 0) {
                    return ret;
                }
                *val = ret;
                return 0;
            default:
                break;
        }
        break;
    case hwmon_power:
            switch (attr) {
                case hwmon_in_input:
                    switch(channel) {
                        case 0:
                            ret = get_electric_data(hxi, SENSOR_12V, SIG_WATTS);
                            break;
                        case 1:
                            ret = get_electric_data(hxi, SENSOR_5V, SIG_WATTS);
                            break;
                        case 2:
                            ret = get_electric_data(hxi, SENSOR_3V3, SIG_WATTS);
                            break;
                        case 3:
                            ret = get_electric_data(hxi, SENSOR_TOTAL, SIG_TOTAL_WATTS);
                            break;
                    }
                    if (ret < 0) {
                        return ret;
                    }
                    *val = ret * 1000; //not sure why this needs additional scaling
                    return 0;
                default:
                    break;
            }
            break;

//	case hwmon_fan:
//		switch (attr) {
//		case hwmon_fan_input:
//			ret = -1; //get_electric_data(hxi, CTL_GET_FAN_RPM, channel, true);
//			if (ret < 0)
//				return ret;
//			*val = ret;
//			return 0;
//		case hwmon_fan_target:
//			/* how to read target values from the device is unknown */
//			/* driver returns last set value or 0			*/
//			if (hxi->target[channel] < 0)
//				return -ENODATA;
//			*val = hxi->target[channel];
//			return 0;
//		default:
//			break;
//		}
//		break;
//	case hwmon_pwm:
//		switch (attr) {
//		case hwmon_pwm_input:
//			ret = -1; //get_electric_data(hxi, CTL_GET_FAN_PWM, channel, false);
//			if (ret < 0)
//				return ret;
//			*val = DIV_ROUND_CLOSEST(ret * 255, 100);
//			return 0;
//		default:
//			break;
//		}
//		break;

	default:
		break;
	}

	return -EOPNOTSUPP;
};

static int hxi_write(struct device *dev, enum hwmon_sensor_types type,
		     u32 attr, int channel, long val)
{
	struct hxi_device *hxi = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			return -1; //set_pwm(hxi, channel, val);
		default:
			break;
		}
		break;
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_target:
			return -1; //set_target(hxi, channel, val);
		default:
			break;
		}
	default:
		break;
	}

	return -EOPNOTSUPP;
};

static umode_t hxi_is_visible(const void *data, enum hwmon_sensor_types type, u32 attr, int channel)
{
	return 0444;
};

static const struct hwmon_ops hxi_hwmon_ops = {
	.is_visible = hxi_is_visible,
	.read = hxi_read,
	.read_string = hxi_read_string,
	.write = hxi_write,
};

static const struct hwmon_channel_info *hxi_info[] = {
	HWMON_CHANNEL_INFO(chip, HWMON_C_REGISTER_TZ),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT, HWMON_T_INPUT
			   ),
//	HWMON_CHANNEL_INFO(fan,
//			   HWMON_F_INPUT | HWMON_F_LABEL | HWMON_F_TARGET
//			   ),
//	HWMON_CHANNEL_INFO(pwm,
//			   HWMON_PWM_INPUT
//			   ),
	HWMON_CHANNEL_INFO(in, HWMON_I_INPUT, HWMON_I_INPUT, HWMON_I_INPUT, HWMON_I_INPUT),
    HWMON_CHANNEL_INFO(curr, HWMON_I_INPUT, HWMON_I_INPUT, HWMON_I_INPUT),
    HWMON_CHANNEL_INFO(power, HWMON_I_INPUT, HWMON_I_INPUT, HWMON_I_INPUT, HWMON_I_INPUT),
	NULL
};

static const struct hwmon_chip_info hxi_chip_info = {
	.ops = &hxi_hwmon_ops,
	.info = hxi_info,
};

static int hxi_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct hxi_device *hxi;
	int ret;

	hxi = devm_kzalloc(&hdev->dev, sizeof(*hxi), GFP_KERNEL);
	if (!hxi)
		return -ENOMEM;

	hxi->buffer = devm_kmalloc(&hdev->dev, OUT_BUFFER_SIZE, GFP_KERNEL);
	if (!hxi->buffer)
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

	hxi->hdev = hdev;
	hid_set_drvdata(hdev, hxi);
	mutex_init(&hxi->mutex);
	init_completion(&hxi->wait_input_report);

	hid_device_io_start(hdev);

	hxi->hwmon_dev = hwmon_device_register_with_info(&hdev->dev, "hxipsu",
							 hxi, &hxi_chip_info, 0);
	if (IS_ERR(hxi->hwmon_dev)) {
		ret = PTR_ERR(hxi->hwmon_dev);
		goto out_hw_close;
	}

	return 0;

out_hw_close:
	hid_hw_close(hdev);
out_hw_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void hxi_remove(struct hid_device *hdev)
{
	struct hxi_device *hxi = hid_get_drvdata(hdev);

	hwmon_device_unregister(hxi->hwmon_dev);
	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static const struct hid_device_id hxi_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_CORSAIR, USB_PRODUCT_ID_CORSAIR_HX850i) },
	//{ HID_USB_DEVICE(USB_VENDOR_ID_CORSAIR, USB_PRODUCT_ID_CORSAIR_1000D) },
	{ }
};

static struct hid_driver hxi_driver = {
	.name = "corsair-hxi",
	.id_table = hxi_devices,
	.probe = hxi_probe,
	.remove = hxi_remove,
	.raw_event = hxi_raw_event,
};

MODULE_DEVICE_TABLE(hid, hxi_devices);
MODULE_LICENSE("GPL");

static int __init hxi_init(void)
{
	return hid_register_driver(&hxi_driver);
}

static void __exit hxi_exit(void)
{
	hid_unregister_driver(&hxi_driver);
}

/*
 * When compiling this driver as built-in, hwmon initcalls will get called before the
 * hid driver and this driver would fail to register. late_initcall solves this.
 */
late_initcall(hxi_init);
module_exit(hxi_exit);
