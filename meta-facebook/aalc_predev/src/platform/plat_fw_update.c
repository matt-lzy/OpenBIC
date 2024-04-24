#include <string.h>
#include <stdlib.h>
#include "plat_version.h"
#include "plat_modbus.h"
#include "modbus_server.h"
#include "util_spi.h"
#include "sensor.h"
#include "nct7363.h"
#include "plat_sensor_table.h"
#include "plat_fw_update.h"

#define UPADTE_FW_DATA_LENGTH_MIN 3 // contain 2 regs(offeset)+ 1 reg(length) at least

uint8_t modbus_get_fw_reversion(modbus_command_mapping *cmd)
{
	uint16_t byte_val = FIRMWARE_REVISION_1 << 8 | FIRMWARE_REVISION_2;
	memcpy(cmd->data, &byte_val, sizeof(uint16_t) * cmd->size);
	return MODBUS_EXC_NONE;
}

uint8_t modbus_fw_download(modbus_command_mapping *cmd)
{
	uint32_t offset = cmd->data[0] << 16 | cmd->data[1]; // offset
	uint16_t msg_len = cmd->data[2] & 0x7FFF; // length
	uint8_t flag = (cmd->data[2] & (1 << 15)) ? (1 << 7) : 0;

	if (cmd->data_len < UPADTE_FW_DATA_LENGTH_MIN)
		return MODBUS_EXC_ILLEGAL_DATA_VAL;

	if (msg_len != ((cmd->data_len - UPADTE_FW_DATA_LENGTH_MIN) * 2))
		return MODBUS_EXC_ILLEGAL_DATA_VAL;

	return fw_update(offset, msg_len, (uint8_t *)&cmd->data[UPADTE_FW_DATA_LENGTH_MIN], flag,
			 DEVSPI_FMC_CS0);
}

uint8_t all_fan_full_duty()
{
	const uint8_t fan_maxnum = 20;
	const uint8_t fan_sensor_numbers[20] = {
		SENSOR_NUM_FB_1_FAN_TACH_RPM,	SENSOR_NUM_FB_2_FAN_TACH_RPM,
		SENSOR_NUM_FB_3_FAN_TACH_RPM,	SENSOR_NUM_FB_4_FAN_TACH_RPM,
		SENSOR_NUM_FB_5_FAN_TACH_RPM,	SENSOR_NUM_FB_6_FAN_TACH_RPM,
		SENSOR_NUM_FB_7_FAN_TACH_RPM,	SENSOR_NUM_FB_8_FAN_TACH_RPM,
		SENSOR_NUM_FB_9_FAN_TACH_RPM,	SENSOR_NUM_FB_10_FAN_TACH_RPM,
		SENSOR_NUM_FB_11_FAN_TACH_RPM,	SENSOR_NUM_FB_12_FAN_TACH_RPM,
		SENSOR_NUM_FB_13_FAN_TACH_RPM,	SENSOR_NUM_FB_14_FAN_TACH_RPM,
		SENSOR_NUM_PB_1_FAN_1_TACH_RPM, SENSOR_NUM_PB_1_FAN_2_TACH_RPM,
		SENSOR_NUM_PB_2_FAN_1_TACH_RPM, SENSOR_NUM_PB_2_FAN_2_TACH_RPM,
		SENSOR_NUM_PB_3_FAN_1_TACH_RPM, SENSOR_NUM_PB_3_FAN_2_TACH_RPM,
	};

	for (int j = 0; j < fan_maxnum; j++) {
		//if (plat_fan_config[j] == NULL)
		sensor_cfg *fan_cfg = get_common_sensor_cfg_info(fan_sensor_numbers[j]);
		if (fan_cfg == NULL)
			return MODBUS_EXC_SERVER_DEVICE_FAILURE;
		else {
			for (int i = 0; i < 16; i++) {
				if (nct7363_set_duty(fan_cfg, 100, i))
					return MODBUS_EXC_SERVER_DEVICE_FAILURE;
			}
		}
	}

	return MODBUS_EXC_NONE;
}