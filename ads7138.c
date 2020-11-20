// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for TI ADS7138 8-Channel 12-Bit A/D converter
 *
 * Copyright (C) 2020 Yuri Nesterov <yuri.nesterov@gmail.com>
 *
 */

#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>

/* ADS7138 opcodes */
#define ADS7138_OPCODE_READ		0x10
#define ADS7138_OPCODE_WRITE	0x08

/* ADS7138 registers */
#define ADS7138_REG_SYSTEM_STATUS	0x0
#define ADS7138_REG_GENERAL_CFG		0x1
#define ADS7138_REG_DATA_CFG		0x2
#define ADS7138_REG_OSR_CFG			0x3
#define ADS7138_REG_OPMODE_CFG		0x4
#define ADS7138_REG_PIN_CFG			0x5
#define ADS7138_REG_SEQUENCE_CFG	0x10
#define ADS7138_REG_CHANNEL_SEL		0x11

/* GENERAL_CFG Register Fields */
#define ADS7138_GENERAL_CFG_RST		0x1

/* DATA_CFG Register Fields */
#define ADS7138_DATA_CFG_FIX_PAT 	0x80
#define ADS7138_DATA_CFG_CH_ID	 	0x10

/* OSR_CFG Register Fields */
#define ADS7138_OSR_CFG_NO_AVG	 	0x0
#define ADS7138_OSR_CFG_2	 		0x1

/* AVDD (VREF) operating range */
#define ADS7138_VREF_MV_MIN	2350
#define ADS7138_VREF_MV_MAX	5500

/* Supported devices */
enum ads7138_chips { ads7138 };

/* ADS7138 data */
struct ads7138_data {
	struct i2c_client *client;
	struct mutex update_lock;
	unsigned int lsb;
};


s32 ads7138_read_reg(const struct ads7138_data *data, u8 reg)
{
	int ret;

	ret = i2c_smbus_write_byte_data(data->client, ADS7138_OPCODE_READ, reg);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_byte(data->client);
	return ret;	
}

s32 ads7138_write_reg(const struct ads7138_data *data, u8 reg, u8 value)
{
	int ret;
	u16 svalue = reg | (value << 8);

	ret = i2c_smbus_write_word_data(data->client, ADS7138_OPCODE_WRITE, svalue);	
	return ret;	
}

s32 ads7138_read_word(const struct ads7138_data *data)
{	
	char rxbuf[2];
	int ret;
	struct i2c_msg msgs[] = {		
		{
			.addr = data->client->addr,
			.flags = I2C_M_RD,
			.len = 2,
			.buf = rxbuf,
		},
	};	

	/* We can't use SMBus here because we need to read 2 bytes with no address write */ 
	ret = i2c_transfer(data->client->adapter, msgs, 1);
	if (ret < 0)
		return ret;
	
	return rxbuf[1] | (rxbuf[0] << 8);
}

/* Sysfs callback function */
static ssize_t ads7138_in_show(struct device *dev,
			       struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ads7138_data *data = dev_get_drvdata(dev);
	int ret;
	int ch;
	int value;

	mutex_lock(&data->update_lock);

	/* Select ADC channel and read the data */
	ret = ads7138_write_reg(data, ADS7138_REG_CHANNEL_SEL, attr->index);
	if (ret < 0)
		dev_err(dev, "Failed to select channel %d\n", attr->index);
	else {	
		ret = ads7138_read_word(data);
		if (ret < 0)
			dev_err(dev, "Failed to read from channel %d\n", attr->index);		
	}

	mutex_unlock(&data->update_lock);
	if (ret < 0)
		return ret;	

	/* Get channel id and sample value */
	ch = ret & 0xf;
	value = le16_to_cpu((ret >> 4) & 0xfff);

	dev_dbg(dev, "ADC data 0x%x ch %d value 0x%x\n", ret, ch, value);

	/* Verify channel id */
	if (ch != attr->index) {

		dev_err(dev, "Channel id mismatch, requested %d received %d\n", attr->index, ch);
		return -EINVAL;
	}

	/* Convert to mV and print out */
	return sprintf(buf, "%d\n", DIV_ROUND_CLOSEST(value * data->lsb, 1000));
}

static SENSOR_DEVICE_ATTR_RO(in0_input, ads7138_in, 0);
static SENSOR_DEVICE_ATTR_RO(in1_input, ads7138_in, 1);
static SENSOR_DEVICE_ATTR_RO(in2_input, ads7138_in, 2);
static SENSOR_DEVICE_ATTR_RO(in3_input, ads7138_in, 3);
static SENSOR_DEVICE_ATTR_RO(in4_input, ads7138_in, 4);
static SENSOR_DEVICE_ATTR_RO(in5_input, ads7138_in, 5);
static SENSOR_DEVICE_ATTR_RO(in6_input, ads7138_in, 6);
static SENSOR_DEVICE_ATTR_RO(in7_input, ads7138_in, 7);

static struct attribute *ads7138_attrs[] = {
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in5_input.dev_attr.attr,
	&sensor_dev_attr_in6_input.dev_attr.attr,
	&sensor_dev_attr_in7_input.dev_attr.attr,
	NULL
};

ATTRIBUTE_GROUPS(ads7138);

static int ads7138_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ads7138_data *data;
	struct device *hwmon_dev;
	int vref_uv = 0;
	int vref_mv = 0;
	int status;
	int ret;
	struct regulator *reg;
	
	data = devm_kzalloc(dev, sizeof(struct ads7138_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	
	data->client = client;
	mutex_init(&data->update_lock);

	/* Get AVDD (Vref) */
	reg = devm_regulator_get(dev, "AVDD");
	if (IS_ERR(reg)) {
		dev_err(dev, "Failed to get AVDD (Vref)\n");
		return PTR_ERR(reg);
	}
	else {
		ret = regulator_enable(reg);
		if (ret) {
			dev_err(dev, "Failed to enable AVDD supply: %d\n", ret);
			return ret;
		}

		vref_uv = regulator_get_voltage(reg);
		vref_mv = DIV_ROUND_CLOSEST(vref_uv, 1000);
	}

	if (vref_mv < ADS7138_VREF_MV_MIN ||
			    vref_mv > ADS7138_VREF_MV_MAX) {
		dev_err(dev, "AVDD (Vref) is not in range\n");
		return -EINVAL;
	}

	/* Calculate LSB */
	vref_mv = clamp_val(vref_mv, ADS7138_VREF_MV_MIN, ADS7138_VREF_MV_MAX);
	data->lsb = DIV_ROUND_CLOSEST(vref_mv * 1000, 4096);

	mutex_lock(&data->update_lock);

	/* Software reset */
	ret = ads7138_write_reg(data, ADS7138_REG_GENERAL_CFG, ADS7138_GENERAL_CFG_RST);
	if (ret < 0) {
		dev_err(dev, "Failed to reset\n");
		goto out;
	}

	/* Get system status */
	status = ads7138_read_reg(data, ADS7138_REG_SYSTEM_STATUS);
	if (status < 0)	{
		dev_err(dev, "Failed to read system status\n");
		goto out;
	}

	dev_dbg(dev, "ads7138 status 0x%x\n", status);	

	/* Set data configuration (enable channel id field) */
	ret = ads7138_write_reg(data, ADS7138_REG_DATA_CFG, ADS7138_DATA_CFG_CH_ID);
	if (ret < 0) {
		dev_err(dev, "Failed to set data configuration\n");
		goto out;
	}
	
	/* Configure all channels as analog input */
	ret = ads7138_write_reg(data, ADS7138_REG_PIN_CFG, 0);
	if (ret < 0) {
		dev_err(dev, "Failed to set pin configuration\n");
		goto out;
	}

	/* Manual mode, conversions are initiated by host */
	ret = ads7138_write_reg(data, ADS7138_REG_OPMODE_CFG, 0);
	if (ret < 0) {
		dev_err(dev, "Failed to set opmode\n");
		goto out;
	}

	/* Manual sequence mode */
	ret = ads7138_write_reg(data, ADS7138_REG_SEQUENCE_CFG, 0);
	if (ret < 0) {
		dev_err(dev, "Failed to set opmode\n");
		goto out;
	}

out:
	mutex_unlock(&data->update_lock);

	if (ret < 0)
		return ret;

	hwmon_dev = devm_hwmon_device_register_with_groups(dev, client->name,
							   data, ads7138_groups);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct i2c_device_id ads7138_device_ids[] = {
	{ "ads7138", ads7138 },	
	{ }
};
MODULE_DEVICE_TABLE(i2c, ads7138_device_ids);

static const struct of_device_id __maybe_unused ads7138_of_match[] = {
	{
		.compatible = "ti,ads7138",
		.data = (void *)ads7138
	},	
	{ },
};
MODULE_DEVICE_TABLE(of, ads7138_of_match);

static struct i2c_driver ads7138_driver = {
	.driver = {
		.name = "ads7138",
		.of_match_table = of_match_ptr(ads7138_of_match),
	},

	.id_table = ads7138_device_ids,
	.probe = ads7138_probe,
};

module_i2c_driver(ads7138_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yuri Nesterov <yuri.nesterov@gmail.com>");
MODULE_DESCRIPTION("Driver for TI ADS7138 A/D converter");
