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
#include "plat_modbus.h"
#include "plat_log.h"
#include "modbus_server.h"
#include <logging/log.h>
#include <libutil.h>
#include "plat_util.h"
#include "plat_pwm.h"
#include "plat_sensor_table.h"

LOG_MODULE_REGISTER(plat_log);

static modbus_err_log_mapping err_log_data[20];

uint16_t error_log_count()
{
	uint16_t count = 0;
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

uint8_t modbus_error_log_event(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_EXC_ILLEGAL_DATA_VAL);

	uint16_t count = (cmd->start_addr - MODBUS_ERR_LOG_1) / cmd->cmd_size;
	err_log_data[count].index = err_log_data[count].index & 0X00FF;

	memcpy(cmd->data, &err_log_data[count], sizeof(uint16_t) * cmd->cmd_size);
	return MODBUS_EXC_NONE;
}

void error_log_event(uint16_t err_code)
{
	uint16_t new_count = 0;
	for (uint16_t count = 0; count < ARRAY_SIZE(err_log_data); count++) {
		if (err_log_data[count].index & BIT(15)) {
			err_log_data[count].index = err_log_data[count].index & 0X00FF;
			new_count = count + 1;
		}
	}

	if (new_count >= ARRAY_SIZE(err_log_data))
		new_count = 0;

	err_log_data[new_count].index = (new_count + 1) | BIT(15);
	err_log_data[new_count].err_code = err_code;
	err_log_data[new_count].sys_time = k_cycle_get_32() - system_uptime();
	err_log_data[new_count].pump_duty = (uint16_t)pump_pwm_dev_duty_setting();
	err_log_data[new_count].fan_duty = (uint16_t)fan_pwm_dev_duty_setting();

	err_log_data[new_count].outlet_temp =
		get_sensor_reading_to_modbus_val(SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C, -2, 1);
	err_log_data[new_count].outlet_press =
		get_sensor_reading_to_modbus_val(SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA, 1, 1);
	err_log_data[new_count].flow_rate =
		get_sensor_reading_to_modbus_val(SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM, 0, 1);
	err_log_data[new_count].volt =
		get_sensor_reading_to_modbus_val(SENSOR_NUM_BPB_HSC_P48V_VIN_VOLT_V, -2, 1);
}