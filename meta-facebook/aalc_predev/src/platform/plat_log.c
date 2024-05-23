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

#include <kernel.h>
#include <stdlib.h>
#include "plat_log.h"
#include "modbus_server.h"
#include <logging/log.h>
#include <libutil.h>
#include "plat_modbus.h"
#include "plat_util.h"
#include "plat_pwm.h"
#include "plat_sensor_table.h"
#include "fru.h"
#include "plat_fru.h"

LOG_MODULE_REGISTER(plat_log);

static const uint8_t log_data_lengh = 20;
static modbus_err_log_mapping err_log_data[20]; // total log amount: 20
static uint8_t err_sensor_caches[19]; // amount of LOG_ERROR_CODE

#define AALC_FRU_LOG_START 0x0200
#define AALC_FRU_LOG_SIZE 0x0C8

const err_sensor_mapping sensor_err_codes[] = {
	{ LEAK_RPU_INT, SENSOR_NUM_BPB_CDU_COOLANT_LEAKAGE_VOLT_V },
	{ LEAK_MAN_PAN_GPO, SENSOR_NUM_BPB_RACK_COOLANT_LEAKAGE_VOLT_V },
	{ LOW_WATER_LEVEL, SENSOR_NUM_BPB_RACK_LEVEL_2 },
	{ PUMP_1_SPEED_ABNORMAL, SENSOR_NUM_PB_1_PUMP_TACH_RPM },
	{ PUMP_2_SPEED_ABNORMAL, SENSOR_NUM_PB_2_PUMP_TACH_RPM },
	{ PUMP_3_SPEED_ABNORMAL, SENSOR_NUM_PB_3_PUMP_TACH_RPM },
};

const err_sensor_mapping sensor_normal_codes[] = {
	{ PUMP_1_SPEED_RECOVER, SENSOR_NUM_PB_1_PUMP_TACH_RPM },
	{ PUMP_2_SPEED_RECOVER, SENSOR_NUM_PB_2_PUMP_TACH_RPM },
	{ PUMP_3_SPEED_RECOVER, SENSOR_NUM_PB_3_PUMP_TACH_RPM },
};

static uint16_t error_log_count(void)
{
	uint16_t count;
	for (count = 0; count < ARRAY_SIZE(err_log_data); count++) {
		if (!err_log_data[count].index)
			return count;
	}
	return (uint16_t)ARRAY_SIZE(err_log_data);
}

uint8_t modbus_error_log_count(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	cmd->data[0] = error_log_count();
	return MODBUS_EXC_NONE;
}

static uint16_t newest_log_event_count(uint8_t order)
{
	if (order > ARRAY_SIZE(err_log_data)) {
		LOG_ERR("Invalid LOG count  %d", order);
		return 0;
	}

	uint16_t newest_count = 0;
	bool is_reset = false;
	for (uint16_t count = 0; count < ARRAY_SIZE(err_log_data); count++) {
		if (err_log_data[count].index == 0XFFFF) {
			is_reset = true;
			newest_count = count;
			break;
		}
	}

	if (is_reset) {
		for (uint16_t count = 0; count < ARRAY_SIZE(err_log_data) - 1; count++) {
			if (err_log_data[count].index <= ARRAY_SIZE(err_log_data)) {
				newest_count = count;
				if (err_log_data[count].index < err_log_data[count + 1].index &&
				    err_log_data[count + 1].index <= ARRAY_SIZE(err_log_data))
					newest_count = count + 1;
				else
					break;
			}
		}
	} else {
		for (uint16_t count = 0; count < ARRAY_SIZE(err_log_data) - 1; count++) {
			if (err_log_data[count].index < err_log_data[count + 1].index)
				newest_count = count + 1;
			else {
				newest_count = count;
				break;
			}
		}
	}

	return (newest_count + ARRAY_SIZE(err_log_data) - (order - 1)) % ARRAY_SIZE(err_log_data);
}

uint8_t modbus_error_log_event(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint16_t order = 1 + ((cmd->start_addr - MODBUS_ERR_LOG_1) / cmd->cmd_size);

	memcpy(cmd->data, &err_log_data[newest_log_event_count(order)],
	       sizeof(uint16_t) * cmd->cmd_size);

	regs_reverse(cmd->data_len, cmd->data);
	return MODBUS_EXC_NONE;
}

bool modbus_clear_log(void)
{
	memset(err_log_data, 0, sizeof(err_log_data));
	memset(err_sensor_caches, 0, sizeof(err_sensor_caches));
	EEPROM_ENTRY fru_entry;

	fru_entry.config.dev_id = MB_FRU_ID; //fru id
	fru_entry.offset = AALC_FRU_LOG_START;
	fru_entry.data_len = AALC_FRU_LOG_SIZE;
	if (FRU_write(&fru_entry)) {
		LOG_ERR("Clear EEPROM Log failed");
		return false;
	} else {
		memcpy(&fru_entry.data[0], &err_log_data[0], fru_entry.data_len);
		return true;
	}
}

//systime format(uint32_t) consists of 2 regs, modbus response will revert the order of regs
uint32_t systime_reverse_regs(void)
{
	uint32_t sys_time = (uint32_t)((k_uptime_get() / 1000) % INT32_MAX);
	return ((sys_time >> 16) & 0xFFFF) | ((sys_time << 16) & 0xFFFF0000);
}

void error_log_event(uint8_t sensor_num, bool val_normal)
{
	bool log_todo = false;
	uint16_t err_code = 0;

	if (val_normal) {
		for (uint8_t i = 0; i < ARRAY_SIZE(err_sensor_caches); i++) {
			if (sensor_num == err_sensor_caches[i]) {
				log_todo = true;
				err_sensor_caches[i] = 0;
				for (uint8_t i = 0; i < ARRAY_SIZE(sensor_normal_codes); i++) {
					if (sensor_num == sensor_normal_codes[i].sen_num)
						err_code = sensor_normal_codes[i].err_code;
				}
				break;
			}
		}
	} else {
		log_todo = true;
		for (uint8_t i = 0; i < ARRAY_SIZE(err_sensor_caches); i++) {
			if (sensor_num == err_sensor_caches[i]) {
				log_todo = false;
				break;
			}
		}
		if (log_todo) {
			for (uint8_t i = 0; i < ARRAY_SIZE(err_sensor_caches); i++) {
				if (err_sensor_caches[i] == 0) {
					err_sensor_caches[i] = sensor_num;
					for (uint8_t i = 0; i < ARRAY_SIZE(sensor_err_codes); i++) {
						if (sensor_num == sensor_err_codes[i].sen_num)
							err_code = sensor_err_codes[i].err_code;
					}
					break;
				}
			}
		}
	}

	if (log_todo) {
		uint16_t newest_count = newest_log_event_count(1);
		uint16_t fru_count = (err_log_data[newest_count].index == 0) ?
					     newest_count :
					     ((newest_count + 1) % ARRAY_SIZE(err_log_data));

		err_log_data[fru_count].index = (err_log_data[newest_count].index == 0xFFFF) ?
							1 :
							(err_log_data[newest_count].index + 1);
		err_log_data[fru_count].err_code = err_code;
		err_log_data[fru_count].sys_time = systime_reverse_regs();
		err_log_data[fru_count].pump_duty = (uint16_t)pump_pwm_dev_duty_setting();
		err_log_data[fru_count].fan_duty = (uint16_t)fan_pwm_dev_duty_setting();

		err_log_data[fru_count].outlet_temp = get_sensor_reading_to_modbus_val(
			SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C, -2, 1);
		err_log_data[fru_count].outlet_press = get_sensor_reading_to_modbus_val(
			SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA, 1, 1);
		err_log_data[fru_count].flow_rate = get_sensor_reading_to_modbus_val(
			SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM, 0, 1);
		err_log_data[fru_count].volt =
			get_sensor_reading_to_modbus_val(SENSOR_NUM_BPB_HSC_P48V_VIN_VOLT_V, -2, 1);

		EEPROM_ENTRY fru_entry;

		fru_entry.config.dev_id = MB_FRU_ID; //fru id
		fru_entry.offset = AALC_FRU_LOG_START + fru_count * log_data_lengh;
		fru_entry.data_len = log_data_lengh;

		memcpy(&fru_entry.data[0], &err_log_data[fru_count], fru_entry.data_len);
		if (FRU_write(&fru_entry))
			LOG_ERR("Write Log failed with Error code: %02x", err_code);
	}
}

void pal_load_eeprom_log(void)
{
	EEPROM_ENTRY fru_entry;

	fru_entry.config.dev_id = MB_FRU_ID; //fru id
	fru_entry.offset = AALC_FRU_LOG_START;
	fru_entry.data_len = AALC_FRU_LOG_SIZE;
	if (FRU_read(&fru_entry) != FRU_READ_SUCCESS)
		LOG_ERR("READ Log failed from EEPROM");
	else
		memcpy(&err_log_data[0], &fru_entry.data[0], fru_entry.data_len);

	bool is_empty = true;
	for (uint16_t i = 0; i < log_data_lengh; i++) {
		if (err_log_data[i].index != 0XFFFF) {
			is_empty = false;
			break;
		}
	}

	if(is_empty)
		modbus_clear_log();

}
