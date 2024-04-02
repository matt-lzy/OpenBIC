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
#include "plat_modbus.h"
#include "plat_sensor_table.h"
#include <sys/util.h>
#include <drivers/gpio.h>
#include <modbus/modbus.h>
#include <logging/log.h>
#include "sensor.h"
#include "modbus_server.h"
#include "fru.h"
#include "eeprom.h"
#include "plat_fru.h"
#include <time.h>

LOG_MODULE_REGISTER(plat_modbus);

struct k_thread modbus_server_thread;
K_KERNEL_STACK_MEMBER(modbus_server_thread_stack, MODBUS_SERVER_THREAD_SIZE);

static char server_iface_name[] = "MODBUS0";
//{DT_PROP(DT_INST(1, zephyr_modbus_serial), label)};
static uint16_t regs_wr[16];
static uint16_t regs_rd[16];
static uint8_t cb_num;
static uint16_t cb_addr[2];

modbus_sensor_cfg plat_modbus_sensors[] = {
	/* sensor number,  modbus data addr  */
	{ SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM, RPU_Coolant_Flow_Rate_LPM },
	{ SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C, RPU_Coolant_Outlet_Temp_C },
	{ SENSOR_NUM_BPB_RPU_COOLANT_INLET_TEMP_C, RPU_Coolant_Inlet_Temp_C },
	{ SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA, RPU_Coolant_Outlet_Pressure_kPa },
	{ SENSOR_NUM_BPB_RPU_COOLANT_INLET_P_KPA, RPU_Coolant_Inlet_Pressure_kPa },
	{ 0, RPU_PWR_W },
	{ 0, AALC_TOTAL_PWR_W },
	{ 0, RPU_INPUT_VOLT_V },
	{ SENSOR_NUM_MB_RPU_AIR_INLET_TEMP_C, RPU_Air_Inlet_Temp_C },
	{ 0, RPU_Pump_PWM_TACH_PCT },
	{ SENSOR_NUM_PB_1_PUMP_TACH_RPM, RPU_Pump1_TACH_RPM },
	{ SENSOR_NUM_PB_2_PUMP_TACH_RPM, RPU_Pump2_TACH_RPM },
	{ SENSOR_NUM_PB_3_PUMP_TACH_RPM, RPU_Pump3_TACH_RPM },
	{ 0, RPU_FAN1_STATUS },
	{ 0, RPU_FAN2_STATUS },
	{ SENSOR_NUM_MB_FAN1_TACH_RPM, RPU_FAN1_TACH_RPM },
	{ SENSOR_NUM_MB_FAN2_TACH_RPM, RPU_FAN2_TACH_RPM },
	{ 0, AALC_Cooling_Capacity_W },
	{ 0, RPU_Pump1_STATUS },
	{ 0, RPU_Pump2_STATUS },
	{ 0, RPU_Pump3_STATUS },
	{ 0, RPU_Reservoir_Status },
	{ 0, RPU_LED_Reservoir_Status },
	{ 0, RPU_LED_Leakage_Status },
	{ 0, RPU_LED_Fault_Status },
	{ 0, RPU_LED_Power_Status },
	{ SENSOR_NUM_BB_TMP75_TEMP_C, BB_TMP75_TEMP_C },
	{ SENSOR_NUM_BPB_RPU_OUTLET_TEMP_C, BPB_RPU_OUTLET_TEMP_C },
	{ SENSOR_NUM_PDB_HDC1080DMBR_TEMP_C, PDB_HDC1080DMBR_TEMP_C },
	{ SENSOR_NUM_BB_HSC_P48V_TEMP_C, BB_HSC_P48V_TEMP_C },
	{ SENSOR_NUM_BPB_HSC_P48V_TEMP_C, BPB_HSC_P48V_TEMP_C },
	{ SENSOR_NUM_PB_1_HDC1080DMBR_TEMP_C, PB_1_HDC1080DMBR_TEMP_C },
	{ SENSOR_NUM_PB_2_HDC1080DMBR_TEMP_C, PB_2_HDC1080DMBR_TEMP_C },
	{ SENSOR_NUM_PB_3_HDC1080DMBR_TEMP_C, PB_3_HDC1080DMBR_TEMP_C },
	{ SENSOR_NUM_PB_1_HSC_P48V_TEMP_C, PB_1_HSC_P48V_TEMP_C },
	{ SENSOR_NUM_PB_2_HSC_P48V_TEMP_C, PB_2_HSC_P48V_TEMP_C },
	{ SENSOR_NUM_PB_3_HSC_P48V_TEMP_C, PB_3_HSC_P48V_TEMP_C },
	{ SENSOR_NUM_PB_1_HSC_P48V_VIN_VOLT_V, PB_1_HSC_P48V_VIN_VOLT_V },
	{ SENSOR_NUM_PB_2_HSC_P48V_VIN_VOLT_V, PB_2_HSC_P48V_VIN_VOLT_V },
	{ SENSOR_NUM_PB_3_HSC_P48V_VIN_VOLT_V, PB_3_HSC_P48V_VIN_VOLT_V },
	{ SENSOR_NUM_BB_HSC_P51V_VIN_VOLT_V, BB_HSC_P51V_VIN_VOLT_V },
	{ SENSOR_NUM_BPB_HSC_P51V_VIN_VOLT_V, BPB_HSC_P51V_VIN_VOLT_V },
	{ SENSOR_NUM_BB_HSC_P51V_IOUT_CURR_A, BB_HSC_P51V_IOUT_CURR_A },
	{ SENSOR_NUM_BPB_HSC_P51V_IOUT_CURR_A, BPB_HSC_P51V_IOUT_CURR_A },
	{ SENSOR_NUM_PB_1_HSC_P48V_IOUT_CURR_A, PB_1_HSC_P48V_IOUT_CURR_A },
	{ SENSOR_NUM_PB_2_HSC_P48V_IOUT_CURR_A, PB_2_HSC_P48V_IOUT_CURR_A },
	{ SENSOR_NUM_PB_3_HSC_P48V_IOUT_CURR_A, PB_3_HSC_P48V_IOUT_CURR_A },
	{ SENSOR_NUM_BB_HSC_P51V_PIN_PWR_W, BB_HSC_P51V_PIN_PWR_W },
	{ SENSOR_NUM_BPB_HSC_P51V_PIN_PWR_W, BPB_HSC_P51V_PIN_PWR_W },
	{ SENSOR_NUM_PB_1_HSC_P48V_PIN_PWR_W, PB_1_HSC_P48V_PIN_PWR_W },
	{ 0, Pump_1_Running },
	{ 0, Pump_2_Running  },
	{ 0, Pump_3_Running  },
	{ SENSOR_NUM_PB_2_HSC_P48V_PIN_PWR_W, PB_2_HSC_P48V_PIN_PWR_W },
	{ SENSOR_NUM_PB_3_HSC_P48V_PIN_PWR_W, PB_3_HSC_P48V_PIN_PWR_W },
	{ SENSOR_NUM_PB_1_FAN_1_TACH_RPM, PB_1_FAN_1_TACH_RPM },
	{ SENSOR_NUM_PB_1_FAN_2_TACH_RPM, PB_1_FAN_2_TACH_RPM },
	{ SENSOR_NUM_PB_2_FAN_1_TACH_RPM, PB_2_FAN_1_TACH_RPM },
	{ SENSOR_NUM_PB_2_FAN_2_TACH_RPM, PB_2_FAN_2_TACH_RPM },
	{ SENSOR_NUM_PB_3_FAN_1_TACH_RPM, PB_3_FAN_1_TACH_RPM },
	{ SENSOR_NUM_PB_3_FAN_2_TACH_RPM, PB_3_FAN_2_TACH_RPM },
	{ SENSOR_NUM_BPB_RACK_PRESSURE_3_P_KPA, BPB_RACK_PRESSURE_3_P_KPA },
	{ SENSOR_NUM_BPB_RACK_PRESSURE_4_P_KPA, BPB_RACK_PRESSURE_4_P_KPA },
	{ SENSOR_NUM_BPB_RACK_LEVEL_1, BPB_RACK_LEVEL_1 },
	{ SENSOR_NUM_BPB_RACK_LEVEL_2, BPB_RACK_LEVEL_2 },
	{ SENSOR_NUM_BPB_CDU_LEVEL_3, BPB_CDU_LEVEL_3 },
	{ SENSOR_NUM_MB_HUM_PCT_RH, MB_HUM_PCT_RH },
	{ SENSOR_NUM_PDB_HUM_PCT_RH, PDB_HUM_PCT_RH },
	{ SENSOR_NUM_PB_1_HUM_PCT_RH, PB_1_HUM_PCT_RH },
	{ SENSOR_NUM_PB_2_HUM_PCT_RH, PB_2_HUM_PCT_RH },
	{ SENSOR_NUM_PB_3_HUM_PCT_RH, PB_3_HUM_PCT_RH },
	{ 0, HEX_FAN_PWM_TACH_PCT },
	{ 0, HEX_PWR_W },
	{ 0, HEX_INPUT_VOLT_V },
	{ 0, HEX_INPUT_CURRENT_V },
	{ SENSOR_NUM_FB_1_FAN_TACH_RPM, HEX_FAN1_TACH_RPM },
	{ SENSOR_NUM_FB_2_FAN_TACH_RPM, HEX_FAN2_TACH_RPM },
	{ SENSOR_NUM_FB_3_FAN_TACH_RPM, HEX_FAN3_TACH_RPM },
	{ SENSOR_NUM_FB_4_FAN_TACH_RPM, HEX_FAN4_TACH_RPM },
	{ SENSOR_NUM_FB_5_FAN_TACH_RPM, HEX_FAN5_TACH_RPM },
	{ SENSOR_NUM_FB_6_FAN_TACH_RPM, HEX_FAN6_TACH_RPM },
	{ SENSOR_NUM_FB_7_FAN_TACH_RPM, HEX_FAN7_TACH_RPM },
	{ SENSOR_NUM_FB_8_FAN_TACH_RPM, HEX_FAN8_TACH_RPM },
	{ SENSOR_NUM_FB_9_FAN_TACH_RPM, HEX_FAN9_TACH_RPM },
	{ SENSOR_NUM_FB_10_FAN_TACH_RPM, HEX_FAN10_TACH_RPM },
	{ SENSOR_NUM_SB_HEX_AIR_OUTLET_1_TEMP_C, HEX_Air_Outlet1_Temp_C },
	{ SENSOR_NUM_SB_HEX_AIR_OUTLET_2_TEMP_C, HEX_Air_Outlet2_Temp_C },
	{ SENSOR_NUM_FB_1_HEX_INLET_TEMP_C, HEX_Air_Inlet1_Temp_C },
	{ SENSOR_NUM_FB_2_HEX_INLET_TEMP_C, HEX_Air_Inlet2_Temp_C },
	{ 0, HEX_Water_Inlet_Temp_C },
	{ 0, HEX_Bladder_Level_Status },
	{ SENSOR_NUM_SB_HEX_AIR_OUTLET_3_TEMP_C, SB_HEX_AIR_OUTLET_3_TEMP_C },
	{ SENSOR_NUM_SB_HEX_AIR_OUTLET_4_TEMP_C, SB_HEX_AIR_OUTLET_4_TEMP_C },
	{ SENSOR_NUM_FB_3_HEX_INLET_TEMP_C, FB_3_HEX_INLET_TEMP_C },
	{ SENSOR_NUM_FB_4_HEX_INLET_TEMP_C, FB_4_HEX_INLET_TEMP_C },
	{ SENSOR_NUM_FB_5_HEX_INLET_TEMP_C, FB_5_HEX_INLET_TEMP_C },
	{ SENSOR_NUM_FB_6_HEX_INLET_TEMP_C, FB_6_HEX_INLET_TEMP_C },
	{ SENSOR_NUM_FB_7_HEX_INLET_TEMP_C, FB_7_HEX_INLET_TEMP_C },
	{ SENSOR_NUM_FB_8_HEX_INLET_TEMP_C, FB_8_HEX_INLET_TEMP_C },
	{ SENSOR_NUM_FB_9_HEX_INLET_TEMP_C, FB_9_HEX_INLET_TEMP_C },
	{ SENSOR_NUM_FB_10_HEX_INLET_TEMP_C, FB_10_HEX_INLET_TEMP_C },
	{ SENSOR_NUM_FB_11_HEX_INLET_TEMP_C, FB_11_HEX_INLET_TEMP_C },
	{ SENSOR_NUM_FB_12_HEX_INLET_TEMP_C, FB_12_HEX_INLET_TEMP_C },
	{ SENSOR_NUM_FB_13_HEX_INLET_TEMP_C, FB_13_HEX_INLET_TEMP_C },
	{ SENSOR_NUM_FB_14_HEX_INLET_TEMP_C, FB_14_HEX_INLET_TEMP_C },
	{ SENSOR_NUM_FB_1_HSC_TEMP_C, FB_1_HSC_TEMP_C },
	{ SENSOR_NUM_FB_2_HSC_TEMP_C, FB_2_HSC_TEMP_C },
	{ SENSOR_NUM_FB_3_HSC_TEMP_C, FB_3_HSC_TEMP_C },
	{ SENSOR_NUM_FB_4_HSC_TEMP_C, FB_4_HSC_TEMP_C },
	{ SENSOR_NUM_FB_5_HSC_TEMP_C, FB_5_HSC_TEMP_C },
	{ SENSOR_NUM_FB_6_HSC_TEMP_C, FB_6_HSC_TEMP_C },
	{ SENSOR_NUM_FB_7_HSC_TEMP_C, FB_7_HSC_TEMP_C },
	{ SENSOR_NUM_FB_8_HSC_TEMP_C, FB_8_HSC_TEMP_C },
	{ SENSOR_NUM_FB_9_HSC_TEMP_C, FB_9_HSC_TEMP_C },
	{ SENSOR_NUM_FB_10_HSC_TEMP_C, FB_10_HSC_TEMP_C },
	{ SENSOR_NUM_FB_11_HSC_TEMP_C, FB_11_HSC_TEMP_C },
	{ SENSOR_NUM_FB_12_HSC_TEMP_C, FB_12_HSC_TEMP_C },
	{ SENSOR_NUM_FB_13_HSC_TEMP_C, FB_13_HSC_TEMP_C },
	{ SENSOR_NUM_FB_14_HSC_TEMP_C, FB_14_HSC_TEMP_C },
	{ SENSOR_NUM_FB_1_HSC_P48V_VIN_VOLT_V, FB_1_HSC_P48V_VIN_VOLT_V },
	{ SENSOR_NUM_FB_2_HSC_P48V_VIN_VOLT_V, FB_2_HSC_P48V_VIN_VOLT_V },
	{ SENSOR_NUM_FB_3_HSC_P48V_VIN_VOLT_V, FB_3_HSC_P48V_VIN_VOLT_V },
	{ SENSOR_NUM_FB_4_HSC_P48V_VIN_VOLT_V, FB_4_HSC_P48V_VIN_VOLT_V },
	{ SENSOR_NUM_FB_5_HSC_P48V_VIN_VOLT_V, FB_5_HSC_P48V_VIN_VOLT_V },
	{ SENSOR_NUM_FB_6_HSC_P48V_VIN_VOLT_V, FB_6_HSC_P48V_VIN_VOLT_V },
	{ SENSOR_NUM_FB_7_HSC_P48V_VIN_VOLT_V, FB_7_HSC_P48V_VIN_VOLT_V },
	{ SENSOR_NUM_FB_8_HSC_P48V_VIN_VOLT_V, FB_8_HSC_P48V_VIN_VOLT_V },
	{ SENSOR_NUM_FB_9_HSC_P48V_VIN_VOLT_V, FB_9_HSC_P48V_VIN_VOLT_V },
	{ SENSOR_NUM_FB_10_HSC_P48V_VIN_VOLT_V, FB_10_HSC_P48V_VIN_VOLT_V },
	{ SENSOR_NUM_FB_11_HSC_P48V_VIN_VOLT_V, FB_11_HSC_P48V_VIN_VOLT_V },
	{ SENSOR_NUM_FB_12_HSC_P48V_VIN_VOLT_V, FB_12_HSC_P48V_VIN_VOLT_V },
	{ SENSOR_NUM_FB_13_HSC_P48V_VIN_VOLT_V, FB_13_HSC_P48V_VIN_VOLT_V },
	{ SENSOR_NUM_FB_14_HSC_P48V_VIN_VOLT_V, FB_14_HSC_P48V_VIN_VOLT_V },
	{ SENSOR_NUM_FB_1_HSC_P48V_IOUT_CURR_A, FB_1_HSC_P48V_IOUT_CURR_A },
	{ SENSOR_NUM_FB_2_HSC_P48V_IOUT_CURR_A, FB_2_HSC_P48V_IOUT_CURR_A },
	{ SENSOR_NUM_FB_3_HSC_P48V_IOUT_CURR_A, FB_3_HSC_P48V_IOUT_CURR_A },
	{ SENSOR_NUM_FB_4_HSC_P48V_IOUT_CURR_A, FB_4_HSC_P48V_IOUT_CURR_A },
	{ SENSOR_NUM_FB_5_HSC_P48V_IOUT_CURR_A, FB_5_HSC_P48V_IOUT_CURR_A },
	{ SENSOR_NUM_FB_6_HSC_P48V_IOUT_CURR_A, FB_6_HSC_P48V_IOUT_CURR_A },
	{ SENSOR_NUM_FB_7_HSC_P48V_IOUT_CURR_A, FB_7_HSC_P48V_IOUT_CURR_A },
	{ SENSOR_NUM_FB_8_HSC_P48V_IOUT_CURR_A, FB_8_HSC_P48V_IOUT_CURR_A },
	{ SENSOR_NUM_FB_9_HSC_P48V_IOUT_CURR_A, FB_9_HSC_P48V_IOUT_CURR_A },
	{ SENSOR_NUM_FB_10_HSC_P48V_IOUT_CURR_A, FB_10_HSC_P48V_IOUT_CURR_A },
	{ SENSOR_NUM_FB_11_HSC_P48V_IOUT_CURR_A, FB_11_HSC_P48V_IOUT_CURR_A },
	{ SENSOR_NUM_FB_12_HSC_P48V_IOUT_CURR_A, FB_12_HSC_P48V_IOUT_CURR_A },
	{ SENSOR_NUM_FB_13_HSC_P48V_IOUT_CURR_A, FB_13_HSC_P48V_IOUT_CURR_A },
	{ SENSOR_NUM_FB_14_HSC_P48V_IOUT_CURR_A, FB_14_HSC_P48V_IOUT_CURR_A },
	{ SENSOR_NUM_FB_1_HSC_P48V_PIN_PWR_W, FB_1_HSC_P48V_PIN_PWR_W },
	{ SENSOR_NUM_FB_2_HSC_P48V_PIN_PWR_W, FB_2_HSC_P48V_PIN_PWR_W },
	{ SENSOR_NUM_FB_3_HSC_P48V_PIN_PWR_W, FB_3_HSC_P48V_PIN_PWR_W },
	{ SENSOR_NUM_FB_4_HSC_P48V_PIN_PWR_W, FB_4_HSC_P48V_PIN_PWR_W },
	{ SENSOR_NUM_FB_5_HSC_P48V_PIN_PWR_W, FB_5_HSC_P48V_PIN_PWR_W },
	{ SENSOR_NUM_FB_6_HSC_P48V_PIN_PWR_W, FB_6_HSC_P48V_PIN_PWR_W },
	{ SENSOR_NUM_FB_7_HSC_P48V_PIN_PWR_W, FB_7_HSC_P48V_PIN_PWR_W },
	{ SENSOR_NUM_FB_8_HSC_P48V_PIN_PWR_W, FB_8_HSC_P48V_PIN_PWR_W },
	{ SENSOR_NUM_FB_9_HSC_P48V_PIN_PWR_W, FB_9_HSC_P48V_PIN_PWR_W },
	{ SENSOR_NUM_FB_10_HSC_P48V_PIN_PWR_W, FB_10_HSC_P48V_PIN_PWR_W },
	{ SENSOR_NUM_FB_11_HSC_P48V_PIN_PWR_W, FB_11_HSC_P48V_PIN_PWR_W },
	{ SENSOR_NUM_FB_12_HSC_P48V_PIN_PWR_W, FB_12_HSC_P48V_PIN_PWR_W },
	{ SENSOR_NUM_FB_13_HSC_P48V_PIN_PWR_W, FB_13_HSC_P48V_PIN_PWR_W },
	{ SENSOR_NUM_FB_14_HSC_P48V_PIN_PWR_W, FB_14_HSC_P48V_PIN_PWR_W },
	{ SENSOR_NUM_FB_11_FAN_TACH_RPM, FB_11_FAN_TACH_RPM },
	{ SENSOR_NUM_FB_12_FAN_TACH_RPM, FB_12_FAN_TACH_RPM },
	{ SENSOR_NUM_FB_13_FAN_TACH_RPM, FB_13_FAN_TACH_RPM },
	{ SENSOR_NUM_FB_14_FAN_TACH_RPM, FB_14_FAN_TACH_RPM },
	{ SENSOR_NUM_SB_HEX_PRESSURE_1_P_KPA, SB_HEX_PRESSURE_1_P_KPA },
	{ SENSOR_NUM_SB_HEX_PRESSURE_2_P_KPA, SB_HEX_PRESSURE_2_P_KPA },
	{ SENSOR_NUM_FB_1_HUM_PCT_RH, FB_1_HUM_PCT_RH },
	{ SENSOR_NUM_FB_2_HUM_PCT_RH, FB_2_HUM_PCT_RH },
	{ SENSOR_NUM_FB_3_HUM_PCT_RH, FB_3_HUM_PCT_RH },
	{ SENSOR_NUM_FB_4_HUM_PCT_RH, FB_4_HUM_PCT_RH },
	{ SENSOR_NUM_FB_5_HUM_PCT_RH, FB_5_HUM_PCT_RH },
	{ SENSOR_NUM_FB_6_HUM_PCT_RH, FB_6_HUM_PCT_RH },
	{ SENSOR_NUM_FB_7_HUM_PCT_RH, FB_7_HUM_PCT_RH },
	{ SENSOR_NUM_FB_8_HUM_PCT_RH, FB_8_HUM_PCT_RH },
	{ SENSOR_NUM_FB_9_HUM_PCT_RH, FB_9_HUM_PCT_RH },
	{ SENSOR_NUM_FB_10_HUM_PCT_RH, FB_10_HUM_PCT_RH },
	{ SENSOR_NUM_FB_11_HUM_PCT_RH, FB_11_HUM_PCT_RH },
	{ SENSOR_NUM_FB_12_HUM_PCT_RH, FB_12_HUM_PCT_RH },
	{ SENSOR_NUM_FB_13_HUM_PCT_RH, FB_13_HUM_PCT_RH },
	{ SENSOR_NUM_FB_14_HUM_PCT_RH, FB_14_HUM_PCT_RH },
	{ SENSOR_NUM_BPB_CDU_COOLANT_LEAKAGE_1, LEAK_RPU_INT },
	{ SENSOR_NUM_BPB_RACK_COOLANT_LEAKAGE_2, LEAK_RACK_FLOOR_GPO_and_Relay },
	
};

static int holding_reg_rd(uint16_t addr, uint16_t *reg, uint16_t reg_qty)
{
	if (cb_num > 0 && (addr > cb_addr[0] && addr <= cb_addr[1])) {
		*reg = regs_rd[cb_num - 1];
		cb_num--;
		return MODBUS_READ_WRITE_REGISTER_SUCCESS;
	}

	cb_num = 0;
	memset(&cb_addr, 0, sizeof(cb_addr));
	memset(&regs_rd, 0, sizeof(regs_rd));

	if (addr < 0x9200) { // sensor addr is less than 0x9200
		int reading = 0;
		if ((reg_qty * 2) != sizeof(reading))  
			return MODBUS_NOT_IN_REGISTER_VAL_RANGE;

		for (uint8_t index = 0; index < sensor_config_count; index++) {
			if (plat_modbus_sensors[index].data_addr == addr) {
				uint8_t status =
					get_sensor_reading(sensor_config, sensor_config_count,
							   plat_modbus_sensors[index].sensor_num,
							   &reading, GET_FROM_CACHE);

				if (status == SENSOR_READ_SUCCESS) { //reading type is int(4Bytes = 2 registers)
					regs_rd[0] = reading & 0x0000FFFF;
					regs_rd[1] = (reading >> 8) >> 8;
					cb_num = reg_qty - 1;					
					*reg = regs_rd[cb_num];										
					cb_addr[0] = addr;
					cb_addr[1] = addr + cb_num;										
					return MODBUS_READ_WRITE_REGISTER_SUCCESS;
				} else {
					LOG_ERR("Read Modbus Sensor failed");
					return MODBUS_READ_WRITE_REGISTER_FAIL;
				}
			}
		}

		LOG_ERR("Wrong Modbus Sensor Addr");
		return MODBUS_ADDR_NOT_DEFINITION;
	} else {
		uint8_t status = 0;
		switch (addr) {
		case FRU_FB_PART_ADDR:
		case FRU_MFR_MODEL_ADDR:
		case FRU_MFR_DATE_ADDR:
		case FRU_MFR_SERIEL_ADDR:
		case MODBUS_POWER_RPU_ADDR:
		case FRU_WORKORDER_ADDR:
		case FRU_HW_REVISION_ADDR:
		case FRU_FW_REVISION_ADDR:
		case FRU_TOTAL_UP_TIME_ADDR:
		case FRU_LAST_ON_TIME_ADDR:
		case FRU_HMI_REVISION_ADDR:
		case FRU_NOAH_ARK_CONFIG_ADDR:
		case FRU_HEATEXCHANGER_CONTROLBOX_FPBN_ADDR:
		case FRU_QUANTA_FB_PART_ADDR:
			status = modbus_read_fruid_data(regs_rd, addr, reg_qty);
			if (status == FRU_READ_SUCCESS) {
				cb_num = reg_qty - 1;
				*reg = regs_rd[cb_num];					
				cb_addr[0] = addr;				
				cb_addr[1] = addr + cb_num;			
				return MODBUS_READ_WRITE_REGISTER_SUCCESS;
			} else {
				return MODBUS_READ_WRITE_REGISTER_FAIL;
			}
		default:
			LOG_ERR("Read Modbus Sensor failed");
			return MODBUS_READ_WRITE_REGISTER_FAIL;
		}

		//printk("Holding register read, addr %u, val %u\n", addr, *reg);

		return MODBUS_READ_WRITE_REGISTER_SUCCESS;
	}

	LOG_ERR("Wrong Modbus Sensor Addr");
	return MODBUS_ADDR_NOT_DEFINITION;
}

static int holding_reg_wr(uint16_t addr, uint16_t reg, uint16_t reg_qty)
{
	uint8_t status = 0;
	if (reg_qty > 0) {
		if (cb_num == 0) {
			cb_num = reg_qty;
			cb_addr[1] = addr + cb_num - 1;
			cb_addr[0] = addr;
		}

		if (cb_num >= 0 && (addr >= cb_addr[0] && addr <= cb_addr[1])) {
			regs_wr[cb_num - 1] = reg;
			cb_num--;
			if (cb_num == 0) {
				goto wr_func;
			} else {
				return MODBUS_READ_WRITE_REGISTER_SUCCESS;
			}
		}
	}

	memset(&cb_addr, 0, sizeof(cb_addr));
	memset(&regs_wr, 0, sizeof(regs_wr));
	cb_num = 0;
	return MODBUS_READ_WRITE_REGISTER_FAIL;

	wr_func:
		switch (cb_addr[0]) {
		case FRU_FB_PART_ADDR:
		case FRU_MFR_MODEL_ADDR:
		case FRU_MFR_DATE_ADDR:
		case FRU_MFR_SERIEL_ADDR:
		case MODBUS_POWER_RPU_ADDR:
		case FRU_WORKORDER_ADDR:
		case FRU_HW_REVISION_ADDR:
		case FRU_FW_REVISION_ADDR:
		case FRU_TOTAL_UP_TIME_ADDR:
		case FRU_LAST_ON_TIME_ADDR:
		case FRU_HMI_REVISION_ADDR:
		case FRU_NOAH_ARK_CONFIG_ADDR:
		case FRU_HEATEXCHANGER_CONTROLBOX_FPBN_ADDR:
		case FRU_QUANTA_FB_PART_ADDR:
			status = modbus_write_fruid_data(regs_wr, cb_addr[0], reg_qty);
			memset(&cb_addr, 0, sizeof(cb_addr));
			memset(&regs_wr, 0, sizeof(regs_wr));
			cb_num = 0;			
			if (status == FRU_WRITE_SUCCESS) {
				return MODBUS_READ_WRITE_REGISTER_SUCCESS;
			} else {
				return MODBUS_READ_WRITE_REGISTER_FAIL;
			}
		default:
			memset(&cb_addr, 0, sizeof(cb_addr));
			memset(&regs_wr, 0, sizeof(regs_wr));
			cb_num = 0;		
			LOG_ERR("Read Modbus Sensor failed");
			return MODBUS_ADDR_NOT_DEFINITION;
		}
}

static struct modbus_user_callbacks mbs_cbs = {
	.holding_reg_rd = holding_reg_rd,
	.holding_reg_wr = holding_reg_wr,
};

const static struct modbus_iface_param server_param = {
	.mode = MODBUS_MODE_RTU,
	.server = {
		.user_cb = &mbs_cbs,
		.unit_id = MODBUS_UART_NODE_ADDR,
	},
	.serial = {
		.baud = MODBUS_UART_BAUDRATE_LOW,
		.parity = MODBUS_UART_PARITY,
	},
};

static void modbus_server_handler(void *arug0, void *arug1, void *arug2)
{
	int val;

	val = init_modbus_server(*server_iface_name, server_param);
	if (val != 0) {
		LOG_ERR("Failed to initialize server");
		return;
	}
}

//void sensor_poll_init()
void modbus_server_handler_init(void)
{
	k_thread_create(&modbus_server_thread, modbus_server_thread_stack,
			K_THREAD_STACK_SIZEOF(modbus_server_thread_stack), modbus_server_handler,
			NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&modbus_server_thread, "modbus_server_handler");
	return;
}
