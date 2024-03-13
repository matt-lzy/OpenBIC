/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include "sensor.h"
#include "hal_i2c.h"
#include "libutil.h"
#include <logging/log.h>
#include <nct7363.h>
#include <string.h>
#include <math.h>
LOG_MODULE_REGISTER(dev_nct7363);

uint8_t nct7363_set_threshold(uint8_t bus, uint8_t address, uint8_t port, uint16_t threshold)
{
	I2C_MSG msg = { 0 };
	msg.bus = bus;
	msg.target_addr = address;
	uint8_t retry = 5;
	uint8_t port_offset = port;
	uint8_t threshold_offset_high_byte =
		NCT7363_FAN_COUNT_THRESHOLD_REG_HIGH_BYTE_BASE_OFFSET + port_offset * 2;
	uint8_t threshold_offset_low_byte =
		NCT7363_FAN_COUNT_THRESHOLD_REG_LOW_BYTE_BASE_OFFSET + port_offset * 2;
	uint8_t threshold_low_byte_value = threshold & NCT7363_FAN_LSB_MASK;
	uint8_t threshold_high_byte_value = threshold >> 8;
	// write high byte value
	msg.tx_len = 2;
	msg.data[0] = threshold_offset_high_byte;
	msg.data[1] = threshold_high_byte_value;
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("set NCT7363_threshold_high_byte_value fail");
		return SENSOR_FAIL_TO_ACCESS;
	}
	// write low byte value
	msg.tx_len = 2;
	msg.data[0] = threshold_offset_low_byte;
	msg.data[1] = threshold_low_byte_value;
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("set NCT7363_threshold_low_byte_value fail");
		return SENSOR_FAIL_TO_ACCESS;
	}
	return 0;
}

uint8_t nct7363_set_duty(sensor_cfg *cfg, uint8_t duty)
{
	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	uint8_t retry = 5;
	uint8_t port_offset = cfg->port;
	uint8_t duty_offset = NCT7363_REG_PWM_BASE_OFFSET + port_offset * 2;
	uint8_t step_offset =
		Speed_Control_Portx_Configuration_Register_BASE_OFFSET + (port_offset / 2);
	uint8_t step_mode = 0;
	float duty_in_255 = 0;
	// check duty step mode
	msg.rx_len = 1;
	msg.tx_len = 1;
	msg.data[0] = step_offset;
	if (i2c_master_read(&msg, retry)) {
		return SENSOR_FAIL_TO_ACCESS;
	}
	switch (port_offset % 2) {
	case 0: {
		step_mode = (msg.data[0] >> 2) & 1;
		break;
	}
	case 1: {
		step_mode = (msg.data[0] >> 6) & 1;
		break;
	}
	default:
		LOG_ERR("Unknown error when get step mode");
		break;
	}
	// set duty
	if (step_mode == 0) {
		duty_in_255 = 127 * duty / 100; // 0x7F
	} else {
		duty_in_255 = 255 * duty / 100; // 0xFF
	}
	msg.tx_len = 2;
	msg.data[0] = duty_offset;
	msg.data[1] = (uint8_t)duty_in_255;

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("set NCT7363_FAN_CTRL_SET_DUTY fail");
		return 0xFF; // set duty fail
	}
	return 0;
}
static bool nct7363_write(sensor_cfg *cfg, uint8_t offset, uint8_t val)
{
	I2C_MSG msg = { 0 };
	uint8_t retry = 5;
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;

	msg.data[0] = offset;
	msg.data[1] = val;

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("nct7363 write offset 0x%02x, val 0x%02x fail", offset, val);
		return false;
	}

	return true;
}
static uint8_t nct7363_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	float rpm = 0;
	int gpio_result = 0;
	uint8_t offset = cfg->offset;
	uint8_t port_offset = cfg->arg0;
	uint8_t fan_poles = cfg->arg1;
	uint8_t fan_count_high_byte_offset =
		NCT7363_REG_FAN_COUNT_VALUE_HIGH_BYTE_BASE_OFFSET + port_offset * 2;
	uint8_t fan_count_low_byte_offset =
		NCT7363_REG_FAN_COUNT_VALUE_LOW_BYTE_BASE_OFFSET + port_offset * 2;
	switch (offset) {
	case NCT7363_FAN_SPEED_OFFSET:

		msg.rx_len = 1;
		msg.tx_len = 1;
		msg.data[0] = fan_count_high_byte_offset;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		uint8_t fan_count_high_byte = msg.data[0];

		msg.data[0] = fan_count_low_byte_offset;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		uint8_t fan_count_low_byte = msg.data[0];
		uint16_t fan_count_value =
			(fan_count_high_byte << 5) | (fan_count_low_byte & NCT7363_FAN_LSB_MASK);
		/* count result */
		rpm = 1350000 / ((float)fan_count_value * ((float)fan_poles / 4)); // RPM
		/* return result */
		sensor_val *sval = (sensor_val *)reading;
		sval->integer = (int16_t)rpm;
		sval->fraction = 0;
		return SENSOR_READ_SUCCESS;
	case NCT7363_FAN_STATUS_OFFSET:
		msg.rx_len = 1;
		msg.tx_len = 1;
		int error_flag = 0;
		msg.data[0] = FAN_Status_REG_0_to_7;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		uint8_t fan_status_0_to_7 = msg.data[0];
		msg.data[0] = FAN_Status_REG_8_to_15;
		if (i2c_master_read(&msg, retry)) {
			return SENSOR_FAIL_TO_ACCESS;
		}
		uint8_t fan_status_8_to_15 = msg.data[0];
		uint16_t fan_status = (fan_status_8_to_15 << 8) | (fan_status_0_to_7);
		for (int i = 0; i < 16; i++) {
			if ((fan_status & 1) == 1) {
				LOG_ERR("FAN%d is not working", i);
				error_flag += 1;
			}
			fan_status = fan_status >> 1;
		}
		if (error_flag == 0) {
			return 0;
		} else {
			return -1;
		}
	case NCT7363_GPIO_READ_OFFSET:
		msg.rx_len = 1;
		msg.tx_len = 1;
		if (port_offset >= 0 && port_offset <= 7) {
			msg.data[0] = NCT7363_GPIO0x_INPUT_PORT_REG_OFFSET;
			if (i2c_master_read(&msg, retry)) {
				return SENSOR_FAIL_TO_ACCESS;
			}
			/* get port offset gpio data*/
			gpio_result = (msg.data[0] >> port_offset) & 1;
			/* return result */
			sensor_val *sval = (sensor_val *)reading;
			sval->integer = (int16_t)gpio_result;
			return SENSOR_READ_SUCCESS;
		} else if (port_offset >= 10 && port_offset <= 17) {
			msg.data[0] = NCT7363_GPIO1x_INPUT_PORT_REG_OFFSET;
			if (i2c_master_read(&msg, retry)) {
				return SENSOR_FAIL_TO_ACCESS;
			}
			/* get port offset gpio data*/
			gpio_result = (msg.data[0] >> (port_offset - 10)) & 1;
			/* return result */
			sensor_val *sval = (sensor_val *)reading;
			sval->integer = (int16_t)gpio_result;
			return SENSOR_READ_SUCCESS;
		} else {
			LOG_ERR("Read GPIO port %d error!", port_offset);
			return SENSOR_UNSPECIFIED_ERROR;
		}
	default:
		LOG_ERR("Unknown register offset(%d)", offset);
		return SENSOR_UNSPECIFIED_ERROR;
	}
}

uint8_t nct7363_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	nct7363_init_arg *nct7363_init_arg_data = (nct7363_init_arg *)cfg->init_args;
	uint8_t offset, val;
	/* check pin_type */
	for (uint8_t i = 0; i < 16; i++) {
		if (nct7363_init_arg_data->pin_type[i] == NCT7363_PIN_TPYE_GPIO ||
		    nct7363_init_arg_data->pin_type[i] == NCT7363_PIN_TPYE_PWM ||
		    nct7363_init_arg_data->pin_type[i] == NCT7363_PIN_TPYE_FANIN ||
		    nct7363_init_arg_data->pin_type[i] == NCT7363_PIN_TPYE_RESERVED) {
			continue;
		} else {
			LOG_ERR("Unknown pin_type, pin number(%d)", i);
			return SENSOR_UNSPECIFIED_ERROR;
		}
	}
	/* init_pin_config */
	for (uint8_t i = 0; i < 4; i++) {
		offset = GPIO_00_to_03_Pin_Configuration_REG +
			 i; // Pin_Configuration_REG base offset
		val = (nct7363_init_arg_data->pin_type[3 + 4 * i] << 6) | // 03 07 13 17
		      (nct7363_init_arg_data->pin_type[2 + 4 * i] << 4) | // 02 06 12 16
		      (nct7363_init_arg_data->pin_type[1 + 4 * i] << 2) | // 01 05 11 15
		      (nct7363_init_arg_data->pin_type[4 * i]); // 00 04 10 14

		if (nct7363_write(cfg, offset, val))
			return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	/* init gpio input/output */
	uint8_t offset_gpio0x = 0, offset_gpio1x = 0, val_gpio0x = 0, val_gpio1x = 0;
	offset_gpio0x = GPIO0x_Input_Output_Configuration_REG;
	offset_gpio1x = GPIO1x_Input_Output_Configuration_REG;
	for (uint8_t i = 0; i < 8; i++) {
		val_gpio0x = (val_gpio0x << 1) | nct7363_init_arg_data->gpio_dir[i];
		val_gpio1x = (val_gpio1x << 1) | nct7363_init_arg_data->gpio_dir[i + 8];
	}
	if (nct7363_write(cfg, offset_gpio0x, val_gpio0x))
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	if (nct7363_write(cfg, offset_gpio1x, val_gpio1x))
		return SENSOR_INIT_UNSPECIFIED_ERROR;

	/* set PWM output */
	uint8_t offset_pwm_ctrl_0_7 = 0, offset_pwm_ctrl_8_15 = 0, val_pwm_ctrl_0_7 = 0,
		val_pwm_ctrl_8_15 = 0;
	offset_pwm_ctrl_0_7 = NCT7363_REG_PWM_CTRL_OUTPUT_0_to_7;
	offset_pwm_ctrl_8_15 = NCT7363_REG_PWM_CTRL_OUTPUT_8_to_15;
	for (int i = 0; i < 8; i++) {
		if (nct7363_init_arg_data->pin_type[i] == NCT7363_PIN_TPYE_PWM)
			val_pwm_ctrl_0_7 |= (1 << i);
		if (nct7363_init_arg_data->pin_type[i + 8] == NCT7363_PIN_TPYE_PWM)
			val_pwm_ctrl_8_15 |= (1 << i);
	}
	if (nct7363_write(cfg, offset_pwm_ctrl_0_7, val_pwm_ctrl_0_7))
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	if (nct7363_write(cfg, offset_pwm_ctrl_8_15, val_pwm_ctrl_8_15))
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	/* set FANIN */
	uint8_t offset_fanin_0_7 = 0, offset_fanin_8_15 = 0, val_fanin_0_7 = 0, val_fanin_8_15 = 0;
	offset_fanin_0_7 = NCT7363_REG_FANIN_CTRL1;
	offset_fanin_8_15 = NCT7363_REG_FANIN_CTRL2;
	for (int i = 0; i < 8; i++) {
		if (nct7363_init_arg_data->pin_type[i] == NCT7363_PIN_TPYE_FANIN)
			val_fanin_0_7 |= (1 << i);
		if (nct7363_init_arg_data->pin_type[i + 8] == NCT7363_PIN_TPYE_FANIN)
			val_fanin_8_15 |= (1 << i);
	}
	if (nct7363_write(cfg, offset_fanin_0_7, val_fanin_0_7))
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	if (nct7363_write(cfg, offset_fanin_8_15, val_fanin_8_15))
		return SENSOR_INIT_UNSPECIFIED_ERROR;

	/* set init threshold  */
	for (int i = 0; i < 16; i++) {
		// init threshold high byte value
		offset = NCT7363_FAN_COUNT_THRESHOLD_REG_HIGH_BYTE_BASE_OFFSET + i * 2;
		val = (nct7363_init_arg_data->threshold) >> 8;
		if (nct7363_write(cfg, offset, val))
			return SENSOR_INIT_UNSPECIFIED_ERROR;

		// init threshold low byte value
		offset = NCT7363_FAN_COUNT_THRESHOLD_REG_LOW_BYTE_BASE_OFFSET + i * 2;
		val = (nct7363_init_arg_data->threshold) & NCT7363_FAN_LSB_MASK;
		if (nct7363_write(cfg, offset, val))
			return SENSOR_INIT_UNSPECIFIED_ERROR;

		/*  set init fan duty */
		offset = NCT7363_REG_PWM_BASE_OFFSET + i * 2;
		val = 0x66; // 40% duty for init

		if (nct7363_write(cfg, offset, val))
			return SENSOR_INIT_UNSPECIFIED_ERROR;
	}
	/* set wdt  */
	offset = NCT7363_WDT_REG_OFFSET;
	uint8_t val_wdt = 0;
	if (nct7363_init_arg_data->watchdog_timeout[0]==1){
		val_wdt  = 0b10000000;
	}
	else{
		val_wdt  = 0b00000000;
	}
	uint8_t time = nct7363_init_arg_data->watchdog_timeout[1];
	switch (time)
	{
	case WDT_30_SEC:
		val_wdt |= 0b00001100;
		break;
	case WDT_7dot5_SEC:
		val_wdt |= 0b00001000;
		break;
	case WDT_3dot75_SEC:
		val_wdt |= 0b00000100;
		break;
	case WDT_15_SEC:
		break;
	default:
		LOG_ERR("WDT init fail !");
		return SENSOR_INIT_UNSPECIFIED_ERROR;
		break;
	}
	if (nct7363_write(cfg, offset, val_wdt))
			return SENSOR_INIT_UNSPECIFIED_ERROR;
			
	nct7363_init_arg_data->is_init = true;
	cfg->arg1 = nct7363_init_arg_data->fan_poles;
	cfg->read = nct7363_read;
	return SENSOR_INIT_SUCCESS;
}
