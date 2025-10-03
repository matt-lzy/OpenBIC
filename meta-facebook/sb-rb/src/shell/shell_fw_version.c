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

#include <zephyr.h>
#include <stdlib.h>
#include <shell/shell.h>
#include "plat_cpld.h"
#include "mp2971.h"
#include "mp29816a.h"
#include "raa228249.h"
#include "plat_pldm_fw_update.h"
#include "plat_pldm_sensor.h"
#include "plat_hook.h"

LOG_MODULE_REGISTER(shell_fw_version);

void cmd_get_fw_version_vr(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_warn(shell, "Help: test get_fw_version vr");
		return;
	}

	/* Stop sensor polling */
	set_plat_sensor_polling_enable_flag(false);

	shell_print(shell, "comp_id |                rail name               |version |remain");
	for (int i = COMPNT_VR_1; i <= COMPNT_VR_12; i++) {
		uint8_t sensor_id = 0;
		char sensor_name[MAX_AUX_SENSOR_NAME_LEN] = { 0 };

		if (is_mb_dc_on() == false)
			continue;

		if (!find_sensor_id_and_name_by_firmware_comp_id(i, &sensor_id, sensor_name)) {
			LOG_ERR("Can't find sensor id and name by comp id: 0x%x", i);
			continue;
		}

		sensor_cfg *cfg = get_sensor_cfg_by_sensor_id(sensor_id);
		if (cfg == NULL)
			continue;

		uint32_t version = 0;
		uint16_t remain = 0xFFFF;
		switch (cfg->type) {
		case sensor_dev_mp2971:
			if (!mp2971_get_checksum(cfg->port, cfg->target_addr, &version)) {
				shell_print(shell, "The VR MPS2971 version reading failed");
				continue;
			}
			break;
		case sensor_dev_mp29816a:
			if (!mp29816a_get_fw_version(cfg->port, cfg->target_addr, &version)) {
				shell_print(shell, "The VR MPS29816a version reading failed");
				continue;
			}
			break;
		case sensor_dev_raa228249:
			if (!raa228249_get_crc(cfg->port, cfg->target_addr, &version)) {
				shell_print(shell, "The VR RAA228249 version reading failed");
				continue;
			}
			if (raa228249_get_remaining_wr(cfg->port, cfg->target_addr,
						       (uint8_t *)&remain) < 0) {
				shell_print(shell, "The VR RAA228249 remaining reading failed");
				continue;
			}
			break;
		default:
			shell_print(shell, "Unsupport VR type(%d)", i);
			return;
		}

		if (remain != 0xFFFF) {
			remain = (uint8_t)((remain % 10) | (remain / 10 << 4));
		}

		if (cfg->type == sensor_dev_mp2891 || cfg->type == sensor_dev_mp29816a)
			shell_print(shell, "%-8x|%-40s|    %04x|%04x", i, sensor_name, version,
				    remain);
		else if (cfg->type == sensor_dev_isl69259 || cfg->type == sensor_dev_raa228238 ||
			 cfg->type == sensor_dev_raa228249 || cfg->type == sensor_dev_mp2971)
			shell_print(shell, "%-8x|%-40s|%08x|%04x", i, sensor_name, version, remain);
		else
			shell_print(shell, "not support sensor_dev: %d", cfg->type);
	}

	/* Start sensor polling */
	set_plat_sensor_polling_enable_flag(true);

	return;
}

void cmd_get_fw_version_cpld(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t data[4] = { 0 };
	uint32_t version = 0;

	if (!plat_read_cpld(CPLD_OFFSET_USERCODE, data, 4)) {
		shell_warn(shell, "cpld read 0x%02x fail", CPLD_OFFSET_USERCODE);
		return;
	}

	version = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	shell_print(shell, "The cpld version: %08x", version);
	return;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_get_fw_version_cmd,
			       SHELL_CMD(vr, NULL, "get fw version vr", cmd_get_fw_version_vr),
			       SHELL_CMD(cpld, NULL, "get fw version cpld",
					 cmd_get_fw_version_cpld),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(get_fw_version, &sub_get_fw_version_cmd, "get fw version command", NULL);