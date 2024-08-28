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

#ifndef PLAT_SENSOR_TABLE_H
#define PLAT_SENSOR_TABLE_H

#include <stdint.h>
#include <pmbus.h>
#include "sensor.h"
#include "plat_modbus.h"

enum PLAT_DEF_SENSOR_E {
	PLAT_DEF_SENSOR_RPU_PWR,
	PLAT_DEF_SENSOR_HEX_PWR,
	PLAT_DEF_SENSOR_TOTAL_PWR,
	PLAT_DEF_SENSOR_HEX_CURR,
	PLAT_DEF_SENSOR_HEX_EXTERNAL_Y_FILTER,
	PLAT_DEF_SENSOR_FAN_PRSNT,
};

/* define sensors address(7 bit) */
#define BB_TMP75_ADDR (0x9E >> 1)
#define BPB_TMP75_ADDR (0x90 >> 1)
#define BB_ADM1272_ADDR (0x26 >> 1)
#define BPB_ADM1272_ADDR (0x20 >> 1)
#define BPB_ADS112C_1_ADDR (0x88 >> 1)
#define BPB_ADS112C_2_ADDR (0x8A >> 1)
#define BPB_ADS112C_3_ADDR (0x82 >> 1)
#define BPB_ADS112C_4_ADDR (0x80 >> 1)
#define BPB_BRICK_12V_ADDR (0x40 >> 1)
#define SB_ADS112C_1_ADDR (0x80 >> 1)
#define SB_ADS112C_2_ADDR (0x82 >> 1)
#define SB_ADS112C_3_ADDR (0x88 >> 1)
#define BPB_NCT7363_ADDR (0x40 >> 1)
#define FB_NCT7363_ADDR (0x42 >> 1)
#define PB_NCT7363_ADDR (0x40 >> 1)
#define HDC1080_ADDR (0x80 >> 1)
#define SB_TMP461_1_ADDR (0x90 >> 1)
#define SB_TMP461_2_ADDR (0x92 >> 1)
#define SB_TMP461_3_ADDR (0x94 >> 1)
#define SB_TMP461_4_ADDR (0x96 >> 1)
#define SB_NCT214_1_ADDR (0x30 >> 1)
#define SB_NCT214_2_ADDR (0x52 >> 1)
#define SB_NCT214_3_ADDR (0x98 >> 1)
#define SB_NCT214_4_ADDR (0x32 >> 1)
#define FB_ADM1272_ADDR (0x22 >> 1)
#define PB_ADM1272_ADDR (0x24 >> 1)
#define PDB_INA238_U14_ADDR (0x8A >> 1)
#define PDB_INA238_U15_ADDR (0x82 >> 1)
#define PDB_MAX11617_U99_ADDR (0x6A >> 1) // U15 2nd src
#define PDB_ADS1015_U101_ADDR (0x90 >> 1) //U15 2nd src

/* define sensors offset */
#define TMP75_TEMP_OFFSET 0x00

#define SENSOR_NUM_FB_1_HSC_TEMP_C 0x00
#define SENSOR_NUM_FB_1_HSC_P48V_VIN_VOLT_V 0x01
#define SENSOR_NUM_FB_1_HSC_P48V_IOUT_CURR_A 0x02
#define SENSOR_NUM_FB_1_HSC_P48V_PIN_PWR_W 0x03
#define SENSOR_NUM_FB_2_HSC_TEMP_C 0x04
#define SENSOR_NUM_FB_2_HSC_P48V_VIN_VOLT_V 0x05
#define SENSOR_NUM_FB_2_HSC_P48V_IOUT_CURR_A 0x06
#define SENSOR_NUM_FB_2_HSC_P48V_PIN_PWR_W 0x07
#define SENSOR_NUM_FB_3_HSC_TEMP_C 0x08
#define SENSOR_NUM_FB_3_HSC_P48V_VIN_VOLT_V 0x09
#define SENSOR_NUM_FB_3_HSC_P48V_IOUT_CURR_A 0x0A
#define SENSOR_NUM_FB_3_HSC_P48V_PIN_PWR_W 0x0B
#define SENSOR_NUM_FB_4_HSC_TEMP_C 0x0C
#define SENSOR_NUM_FB_4_HSC_P48V_VIN_VOLT_V 0x0D
#define SENSOR_NUM_FB_4_HSC_P48V_IOUT_CURR_A 0x0E
#define SENSOR_NUM_FB_4_HSC_P48V_PIN_PWR_W 0x0F
#define SENSOR_NUM_FB_5_HSC_TEMP_C 0x10
#define SENSOR_NUM_FB_5_HSC_P48V_VIN_VOLT_V 0x11
#define SENSOR_NUM_FB_5_HSC_P48V_IOUT_CURR_A 0x12
#define SENSOR_NUM_FB_5_HSC_P48V_PIN_PWR_W 0x13
#define SENSOR_NUM_FB_6_HSC_TEMP_C 0x14
#define SENSOR_NUM_FB_6_HSC_P48V_VIN_VOLT_V 0x15
#define SENSOR_NUM_FB_6_HSC_P48V_IOUT_CURR_A 0x16
#define SENSOR_NUM_FB_6_HSC_P48V_PIN_PWR_W 0x17
#define SENSOR_NUM_FB_7_HSC_TEMP_C 0x18
#define SENSOR_NUM_FB_7_HSC_P48V_VIN_VOLT_V 0x19
#define SENSOR_NUM_FB_7_HSC_P48V_IOUT_CURR_A 0x1A
#define SENSOR_NUM_FB_7_HSC_P48V_PIN_PWR_W 0x1B
#define SENSOR_NUM_FB_8_HSC_TEMP_C 0x1C
#define SENSOR_NUM_FB_8_HSC_P48V_VIN_VOLT_V 0x1D
#define SENSOR_NUM_FB_8_HSC_P48V_IOUT_CURR_A 0x1E
#define SENSOR_NUM_FB_8_HSC_P48V_PIN_PWR_W 0x1F
#define SENSOR_NUM_FB_9_HSC_TEMP_C 0x20
#define SENSOR_NUM_FB_9_HSC_P48V_VIN_VOLT_V 0x21
#define SENSOR_NUM_FB_9_HSC_P48V_IOUT_CURR_A 0x22
#define SENSOR_NUM_FB_9_HSC_P48V_PIN_PWR_W 0x23
#define SENSOR_NUM_FB_10_HSC_TEMP_C 0x24
#define SENSOR_NUM_FB_10_HSC_P48V_VIN_VOLT_V 0x25
#define SENSOR_NUM_FB_10_HSC_P48V_IOUT_CURR_A 0x26
#define SENSOR_NUM_FB_10_HSC_P48V_PIN_PWR_W 0x27
#define SENSOR_NUM_FB_11_HSC_TEMP_C 0x28
#define SENSOR_NUM_FB_11_HSC_P48V_VIN_VOLT_V 0x29
#define SENSOR_NUM_FB_11_HSC_P48V_IOUT_CURR_A 0x2A
#define SENSOR_NUM_FB_11_HSC_P48V_PIN_PWR_W 0x2B
#define SENSOR_NUM_FB_12_HSC_TEMP_C 0x2C
#define SENSOR_NUM_FB_12_HSC_P48V_VIN_VOLT_V 0x2D
#define SENSOR_NUM_FB_12_HSC_P48V_IOUT_CURR_A 0x2E
#define SENSOR_NUM_FB_12_HSC_P48V_PIN_PWR_W 0x2F
#define SENSOR_NUM_FB_13_HSC_TEMP_C 0x30
#define SENSOR_NUM_FB_13_HSC_P48V_VIN_VOLT_V 0x31
#define SENSOR_NUM_FB_13_HSC_P48V_IOUT_CURR_A 0x32
#define SENSOR_NUM_FB_13_HSC_P48V_PIN_PWR_W 0x33
#define SENSOR_NUM_FB_14_HSC_TEMP_C 0x34
#define SENSOR_NUM_FB_14_HSC_P48V_VIN_VOLT_V 0x35
#define SENSOR_NUM_FB_14_HSC_P48V_IOUT_CURR_A 0x36
#define SENSOR_NUM_FB_14_HSC_P48V_PIN_PWR_W 0x37
#define SENSOR_NUM_PB_1_HSC_P48V_TEMP_C 0x38
#define SENSOR_NUM_PB_1_HSC_P48V_VIN_VOLT_V 0x39
#define SENSOR_NUM_PB_1_HSC_P48V_IOUT_CURR_A 0x3A
#define SENSOR_NUM_PB_1_HSC_P48V_PIN_PWR_W 0x3B
#define SENSOR_NUM_PB_2_HSC_P48V_TEMP_C 0x3C
#define SENSOR_NUM_PB_2_HSC_P48V_VIN_VOLT_V 0x3D
#define SENSOR_NUM_PB_2_HSC_P48V_IOUT_CURR_A 0x3E
#define SENSOR_NUM_PB_2_HSC_P48V_PIN_PWR_W 0x3F
#define SENSOR_NUM_PB_3_HSC_P48V_TEMP_C 0x40
#define SENSOR_NUM_PB_3_HSC_P48V_VIN_VOLT_V 0x41
#define SENSOR_NUM_PB_3_HSC_P48V_IOUT_CURR_A 0x42
#define SENSOR_NUM_PB_3_HSC_P48V_PIN_PWR_W 0x43
#define SENSOR_NUM_BB_HSC_P48V_TEMP_C 0x44
#define SENSOR_NUM_BB_HSC_P48V_VIN_VOLT_V 0x45
#define SENSOR_NUM_BB_HSC_P48V_IOUT_CURR_A 0x46
#define SENSOR_NUM_BB_HSC_P48V_PIN_PWR_W 0x47
#define SENSOR_NUM_BPB_HSC_P48V_TEMP_C 0x48
#define SENSOR_NUM_BPB_HSC_P48V_VIN_VOLT_V 0x49
#define SENSOR_NUM_BPB_HSC_P48V_IOUT_CURR_A 0x4A
#define SENSOR_NUM_BPB_HSC_P48V_PIN_PWR_W 0x4B
#define SENSOR_NUM_FB_1_FAN_TACH_RPM 0x4C
#define SENSOR_NUM_FB_2_FAN_TACH_RPM 0x4D
#define SENSOR_NUM_FB_3_FAN_TACH_RPM 0x4E
#define SENSOR_NUM_FB_4_FAN_TACH_RPM 0x4F
#define SENSOR_NUM_FB_5_FAN_TACH_RPM 0x50
#define SENSOR_NUM_FB_6_FAN_TACH_RPM 0x51
#define SENSOR_NUM_FB_7_FAN_TACH_RPM 0x52
#define SENSOR_NUM_FB_8_FAN_TACH_RPM 0x53
#define SENSOR_NUM_FB_9_FAN_TACH_RPM 0x54
#define SENSOR_NUM_FB_10_FAN_TACH_RPM 0x55
#define SENSOR_NUM_FB_11_FAN_TACH_RPM 0x56
#define SENSOR_NUM_FB_12_FAN_TACH_RPM 0x57
#define SENSOR_NUM_FB_13_FAN_TACH_RPM 0x58
#define SENSOR_NUM_FB_14_FAN_TACH_RPM 0x59
#define SENSOR_NUM_PB_1_PUMP_TACH_RPM 0x5A
#define SENSOR_NUM_PB_1_FAN_1_TACH_RPM 0x5B
#define SENSOR_NUM_PB_1_FAN_2_TACH_RPM 0x5C
#define SENSOR_NUM_PB_2_PUMP_TACH_RPM 0x5D
#define SENSOR_NUM_PB_2_FAN_1_TACH_RPM 0x5E
#define SENSOR_NUM_PB_2_FAN_2_TACH_RPM 0x5F
#define SENSOR_NUM_PB_3_PUMP_TACH_RPM 0x60
#define SENSOR_NUM_PB_3_FAN_1_TACH_RPM 0x61
#define SENSOR_NUM_PB_3_FAN_2_TACH_RPM 0x62
#define SENSOR_NUM_MB_FAN1_TACH_RPM 0x63
#define SENSOR_NUM_MB_FAN2_TACH_RPM 0x64
#define SENSOR_NUM_BPB_RACK_LEVEL_1 0x65
#define SENSOR_NUM_BPB_RACK_LEVEL_2 0x66
#define SENSOR_NUM_BPB_RPU_COOLANT_INLET_P_KPA 0x67
#define SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA 0x68
#define SENSOR_NUM_BPB_RACK_PRESSURE_3_P_KPA 0x69
#define SENSOR_NUM_BPB_RACK_PRESSURE_4_P_KPA 0x6A
#define SENSOR_NUM_SB_HEX_PRESSURE_1_P_KPA 0x6B
#define SENSOR_NUM_SB_HEX_PRESSURE_2_P_KPA 0x6C
#define SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM 0x6D
#define SENSOR_NUM_FB_1_HEX_OUTLET_TEMP_C 0x6E
#define SENSOR_NUM_FB_2_HEX_OUTLET_TEMP_C 0x6F
#define SENSOR_NUM_FB_3_HEX_OUTLET_TEMP_C 0x70
#define SENSOR_NUM_FB_4_HEX_OUTLET_TEMP_C 0x71
#define SENSOR_NUM_FB_5_HEX_OUTLET_TEMP_C 0x72
#define SENSOR_NUM_FB_6_HEX_OUTLET_TEMP_C 0x73
#define SENSOR_NUM_FB_7_HEX_OUTLET_TEMP_C 0x74
#define SENSOR_NUM_FB_8_HEX_OUTLET_TEMP_C 0x75
#define SENSOR_NUM_FB_9_HEX_OUTLET_TEMP_C 0x76
#define SENSOR_NUM_FB_10_HEX_OUTLET_TEMP_C 0x77
#define SENSOR_NUM_FB_11_HEX_OUTLET_TEMP_C 0x78
#define SENSOR_NUM_FB_12_HEX_OUTLET_TEMP_C 0x79
#define SENSOR_NUM_FB_13_HEX_OUTLET_TEMP_C 0x7A
#define SENSOR_NUM_FB_14_HEX_OUTLET_TEMP_C 0x7B
#define SENSOR_NUM_PB_1_HDC1080DMBR_TEMP_C 0x7C
#define SENSOR_NUM_PB_2_HDC1080DMBR_TEMP_C 0x7D
#define SENSOR_NUM_PB_3_HDC1080DMBR_TEMP_C 0x7E
#define SENSOR_NUM_BB_TMP75_TEMP_C 0x7F
#define SENSOR_NUM_BPB_RPU_OUTLET_TEMP_C 0x80
#define SENSOR_NUM_BPB_RPU_COOLANT_INLET_TEMP_C 0x81
#define SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C 0x82
#define SENSOR_NUM_BPB_HEX_WATER_INLET_TEMP_C 0x83
#define SENSOR_NUM_MB_RPU_AIR_INLET_TEMP_C 0x84
#define SENSOR_NUM_PDB_HDC1080DMBR_TEMP_C 0x85
#define SENSOR_NUM_SB_HEX_AIR_INLET_1_TEMP_C 0x86
#define SENSOR_NUM_SB_HEX_AIR_INLET_2_TEMP_C 0x87
#define SENSOR_NUM_SB_HEX_AIR_INLET_3_TEMP_C 0x88
#define SENSOR_NUM_SB_HEX_AIR_INLET_4_TEMP_C 0x89
#define SENSOR_NUM_FB_1_HUM_PCT_RH 0x8A
#define SENSOR_NUM_FB_2_HUM_PCT_RH 0x8B
#define SENSOR_NUM_FB_3_HUM_PCT_RH 0x8C
#define SENSOR_NUM_FB_4_HUM_PCT_RH 0x8D
#define SENSOR_NUM_FB_5_HUM_PCT_RH 0x8E
#define SENSOR_NUM_FB_6_HUM_PCT_RH 0x8F
#define SENSOR_NUM_FB_7_HUM_PCT_RH 0x90
#define SENSOR_NUM_FB_8_HUM_PCT_RH 0x91
#define SENSOR_NUM_FB_9_HUM_PCT_RH 0x92
#define SENSOR_NUM_FB_10_HUM_PCT_RH 0x93
#define SENSOR_NUM_FB_11_HUM_PCT_RH 0x94
#define SENSOR_NUM_FB_12_HUM_PCT_RH 0x95
#define SENSOR_NUM_FB_13_HUM_PCT_RH 0x96
#define SENSOR_NUM_FB_14_HUM_PCT_RH 0x97
#define SENSOR_NUM_MB_HUM_PCT_RH 0x98
#define SENSOR_NUM_PDB_HUM_PCT_RH 0x99
#define SENSOR_NUM_PB_1_HUM_PCT_RH 0x9A
#define SENSOR_NUM_PB_2_HUM_PCT_RH 0x9B
#define SENSOR_NUM_PB_3_HUM_PCT_RH 0x9C
#define SENSOR_NUM_V_12_AUX 0x9D
#define SENSOR_NUM_V_5_AUX 0x9E
#define SENSOR_NUM_V_3_3_AUX 0x9F
#define SENSOR_NUM_V_1_2_AUX 0xA0
#define SENSOR_NUM_V_5_USB 0xA1
#define SENSOR_NUM_BPB_CDU_COOLANT_LEAKAGE_VOLT_V 0xA2
#define SENSOR_NUM_BPB_RACK_COOLANT_LEAKAGE_VOLT_V 0xA3
#define SENSOR_NUM_SB_TTV_COOLANT_LEAKAGE_1_VOLT_V 0xA4
#define SENSOR_NUM_SB_TTV_COOLANT_LEAKAGE_2_VOLT_V 0xA5
#define SENSOR_NUM_SB_TTV_COOLANT_LEAKAGE_3_VOLT_V 0xA6
#define SENSOR_NUM_PDB_48V_SENSE_DIFF_POS_VOLT_V 0xA7
#define SENSOR_NUM_PDB_48V_SENSE_DIFF_NEG_VOLT_V 0xA8
#define SENSOR_NUM_BPB_BRICK_12V_VIN_VOLT_V 0xA9
#define SENSOR_NUM_BPB_BRICK_12V_VOUT_VOLT_V 0xAA
#define SENSOR_NUM_BPB_BRICK_12V_IOUT_CURR_A 0xAB
#define SENSOR_NUM_BPB_BRICK_12V_TEMP_C 0xAC

// plat def sensor
#define SENSOR_NUM_RPU_PWR_W 0xC0
#define SENSOR_NUM_HEX_PWR_W 0xC1
#define SENSOR_NUM_AALC_TOTAL_PWR_W 0xC2
#define SENSOR_NUM_HEX_CURR_A 0xC3
#define SENSOR_NUM_HEX_EXTERNAL_Y_FILTER 0xC4
#define SENSOR_NUM_FAN_PRSNT 0xC5

//plat def sensor- IT_LEAK
#define SENSOR_NUM_IT_LEAK_0_GPIO 0XD0
#define SENSOR_NUM_IT_LEAK_1_GPIO 0XD1
#define SENSOR_NUM_IT_LEAK_2_GPIO 0XD2
#define SENSOR_NUM_IT_LEAK_3_GPIO 0XD3

uint8_t plat_get_config_size();
void load_sensor_config(void);
uint8_t get_sensor_reading_to_real_val(uint8_t sensor_num, float *val);
uint16_t get_sensor_reading_to_modbus_val(uint8_t sensor_num, int8_t exp, int8_t scale);
bool switch_sensor_mux(sensor_cfg *cfg);
void quick_sensor_poll_init();

#endif
