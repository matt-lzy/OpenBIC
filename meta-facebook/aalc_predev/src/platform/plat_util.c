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

#include <stdlib.h>
#include "plat_modbus.h"
#include "plat_util.h"
#include "modbus_server.h"
#include <logging/log.h>
#include "util_spi.h"
#include "libutil.h"
#include "plat_sensor_table.h"
#include "sensor.h"

#define I2C_MASTER_READ_BACK_MAX_SIZE 16 // 16 registers

static uint16_t temp_read_length;
static uint16_t temp_read_data[I2C_MASTER_READ_BACK_MAX_SIZE];

LOG_MODULE_REGISTER(plat_util);

uint16_t read_sensor_reading(uint8_t sensor_num, uint16_t *reading_data)
{
	int reading = 0;
	uint8_t status = get_sensor_reading(sensor_config, sensor_config_count, sensor_num,
					    &reading, GET_FROM_CACHE);

	if (status == SENSOR_READ_SUCCESS) {
		sensor_val *sval = (sensor_val *)&reading;
		float reading_result = (sval->integer * 1000) + (sval->fraction / 1000);
		*reading_data = (uint16_t)reading_result;
		return READ_SENSOR_SUCCESS;
	}

	LOG_ERR("modbus cmd read sensor 0x%x failed", sensor_num);
	return READ_SENSOR_FAIL;
}

uint8_t modbus_command_read_rpu_pwr(modbus_command_mapping *cmd)
{
	// sum of backplane board and bridge board pwr
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);
	uint8_t read_array[2] = { SENSOR_NUM_BB_HSC_P48V_PIN_PWR_W,
				  SENSOR_NUM_BPB_HSC_P48V_PIN_PWR_W };
	uint16_t rpu_pwr_val = 0, reading_data_back = 0;
	int len = sizeof(read_array) / sizeof(read_array[0]);
	for (int i = 0; i < len; i++) {
		uint8_t reading_pwr_back = read_sensor_reading(read_array[i], &reading_data_back);
		rpu_pwr_val += reading_data_back;
		if (reading_pwr_back == READ_SENSOR_FAIL)
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}

	memcpy(cmd->data, &rpu_pwr_val, sizeof(uint16_t) * cmd->cmd_size);
	return MODBUS_EXC_NONE;
}

uint8_t modbus_command_read_aalc_total_pwr(modbus_command_mapping *cmd)
{
	// sum of rpu total pwr and hex total pwr
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);
	uint8_t read_array[16] = {
		SENSOR_NUM_BB_HSC_P48V_PIN_PWR_W,    SENSOR_NUM_BPB_HSC_P48V_PIN_PWR_W,
		SENSOR_NUM_FB_1_HSC_P48V_PIN_PWR_W,  SENSOR_NUM_FB_2_HSC_P48V_PIN_PWR_W,
		SENSOR_NUM_FB_3_HSC_P48V_PIN_PWR_W,  SENSOR_NUM_FB_4_HSC_P48V_PIN_PWR_W,
		SENSOR_NUM_FB_5_HSC_P48V_PIN_PWR_W,  SENSOR_NUM_FB_6_HSC_P48V_PIN_PWR_W,
		SENSOR_NUM_FB_7_HSC_P48V_PIN_PWR_W,  SENSOR_NUM_FB_8_HSC_P48V_PIN_PWR_W,
		SENSOR_NUM_FB_9_HSC_P48V_PIN_PWR_W,  SENSOR_NUM_FB_10_HSC_P48V_PIN_PWR_W,
		SENSOR_NUM_FB_11_HSC_P48V_PIN_PWR_W, SENSOR_NUM_FB_12_HSC_P48V_PIN_PWR_W,
		SENSOR_NUM_FB_13_HSC_P48V_PIN_PWR_W, SENSOR_NUM_FB_14_HSC_P48V_PIN_PWR_W
	};

	uint16_t rpu_pwr_val = 0, reading_data_back = 0;
	int len = sizeof(read_array) / sizeof(read_array[0]);
	for (int i = 0; i < len; i++) {
		uint8_t reading_pwr_back = read_sensor_reading(read_array[i], &reading_data_back);
		rpu_pwr_val += reading_data_back;
		if (reading_pwr_back == READ_SENSOR_FAIL)
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}

	memcpy(cmd->data, &rpu_pwr_val, sizeof(uint16_t) * cmd->cmd_size);
	return MODBUS_EXC_NONE;
}

uint8_t modbus_command_read_hex_fan_pwr_tach_pct(modbus_command_mapping *cmd)
{
	// TO DO
	return true;
}

uint8_t modbus_command_read_hex_pwr(modbus_command_mapping *cmd)
{
	// sum of hex pwr
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);
	uint8_t read_array[14] = {
		SENSOR_NUM_FB_1_HSC_P48V_PIN_PWR_W,  SENSOR_NUM_FB_2_HSC_P48V_PIN_PWR_W,
		SENSOR_NUM_FB_3_HSC_P48V_PIN_PWR_W,  SENSOR_NUM_FB_4_HSC_P48V_PIN_PWR_W,
		SENSOR_NUM_FB_5_HSC_P48V_PIN_PWR_W,  SENSOR_NUM_FB_6_HSC_P48V_PIN_PWR_W,
		SENSOR_NUM_FB_7_HSC_P48V_PIN_PWR_W,  SENSOR_NUM_FB_8_HSC_P48V_PIN_PWR_W,
		SENSOR_NUM_FB_9_HSC_P48V_PIN_PWR_W,  SENSOR_NUM_FB_10_HSC_P48V_PIN_PWR_W,
		SENSOR_NUM_FB_11_HSC_P48V_PIN_PWR_W, SENSOR_NUM_FB_12_HSC_P48V_PIN_PWR_W,
		SENSOR_NUM_FB_13_HSC_P48V_PIN_PWR_W, SENSOR_NUM_FB_14_HSC_P48V_PIN_PWR_W
	};

	uint16_t hex_pwr_val = 0, reading_data_back = 0;
	int len = sizeof(read_array) / sizeof(read_array[0]);
	for (int i = 0; i < len; i++) {
		uint8_t reading_pwr_back = read_sensor_reading(read_array[i], &reading_data_back);
		hex_pwr_val += reading_data_back;
		if (reading_pwr_back == READ_SENSOR_FAIL)
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}

	memcpy(cmd->data, &hex_pwr_val, sizeof(uint16_t) * cmd->cmd_size);
	return MODBUS_EXC_NONE;
}

uint8_t modbus_command_read_hex_curr(modbus_command_mapping *cmd)
{
	// sum of hex curr
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);
	uint8_t read_array[14] = {
		SENSOR_NUM_FB_1_HSC_P48V_IOUT_CURR_A,  SENSOR_NUM_FB_2_HSC_P48V_IOUT_CURR_A,
		SENSOR_NUM_FB_3_HSC_P48V_IOUT_CURR_A,  SENSOR_NUM_FB_4_HSC_P48V_IOUT_CURR_A,
		SENSOR_NUM_FB_5_HSC_P48V_IOUT_CURR_A,  SENSOR_NUM_FB_6_HSC_P48V_IOUT_CURR_A,
		SENSOR_NUM_FB_7_HSC_P48V_IOUT_CURR_A,  SENSOR_NUM_FB_8_HSC_P48V_IOUT_CURR_A,
		SENSOR_NUM_FB_9_HSC_P48V_IOUT_CURR_A,  SENSOR_NUM_FB_10_HSC_P48V_IOUT_CURR_A,
		SENSOR_NUM_FB_11_HSC_P48V_IOUT_CURR_A, SENSOR_NUM_FB_12_HSC_P48V_IOUT_CURR_A,
		SENSOR_NUM_FB_13_HSC_P48V_IOUT_CURR_A, SENSOR_NUM_FB_14_HSC_P48V_IOUT_CURR_A
	};

	uint16_t hex_curr_val = 0, reading_data_back = 0;
	int len = sizeof(read_array) / sizeof(read_array[0]);
	for (int i = 0; i < len; i++) {
		uint8_t reading_curr_back = read_sensor_reading(read_array[i], &reading_data_back);
		hex_curr_val += reading_data_back;
		if (reading_curr_back == READ_SENSOR_FAIL)
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}

	memcpy(cmd->data, &hex_curr_val, sizeof(uint16_t) * cmd->cmd_size);
	return MODBUS_EXC_NONE;
}

uint8_t modbus_command_i2c_master_write_read(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	// write data: bus(2Bytes), addr(2Bytes), read length(2Bytes), data(26Bytes)

	if (cmd->data_len <= 3) // check bus,addr,read length is not null
		return MODBUS_EXC_ILLEGAL_DATA_VAL;

	const uint8_t target_bus = cmd->data[0] & BIT_MASK(8); // get 7:0 bit data
	const uint8_t target_addr = cmd->data[1] & BIT_MASK(8);
	const uint8_t target_read_length = cmd->data[2] & BIT_MASK(8);
	I2C_MSG msg = { 0 };
	uint8_t retry = 5;
	msg.bus = target_bus;
	msg.target_addr = target_addr;
	msg.tx_len = cmd->data_len - 3; // write length need to -3 (bus,addr,read length)
	for (int i = 0; i < (cmd->data_len - 3); i++)
		msg.data[i] = cmd->data[i + 3] & BIT_MASK(8);

	if (target_read_length == 0) { // only write
		int result = i2c_master_write(&msg, retry);
		if (result != 0) {
			LOG_ERR("I2C write fail \n");
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;
		}
		return MODBUS_EXC_NONE;
	}

	temp_read_length = target_read_length;
	msg.rx_len = (int)temp_read_length;
	int result = i2c_master_read(&msg, retry);
	if (result != 0) {
		LOG_ERR("I2C read fail \n");
		return MODBUS_EXC_SERVER_DEVICE_FAILURE;
	}

	memset(temp_read_data, 0xff, sizeof(temp_read_data));
	for (int i = 0; i < temp_read_length; i++)
		temp_read_data[i] = msg.data[i];

	return MODBUS_EXC_NONE;
}

uint8_t modbus_command_i2c_master_write_read_response(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	// write data: bus(2Bytes), addr(2Bytes), read length(2Bytes), data(reg:2Bytes+data:24Bytes)
	memcpy(cmd->data, temp_read_data, sizeof(uint16_t) * temp_read_length);

	return MODBUS_EXC_NONE;
}

void regs_reverse(uint16_t reg_len, uint16_t *data)
{
	CHECK_NULL_ARG(data);
	for (uint16_t i = 0; i < reg_len; i++)
		data[i] = sys_be16_to_cpu(data[i]);
}