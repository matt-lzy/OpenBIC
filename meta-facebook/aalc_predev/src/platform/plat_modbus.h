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

#include <stdbool.h>
#include <stdint.h>
#include <kernel.h>

#define MODBUS_SERVER_THREAD_SIZE 3056

#define MODBUS_UART_NODE_ADDR 0x0C
#define MODBUS_UART_FP_OFFSET 5000

void modbus_server_handler_init(void);

typedef struct _modbus_sensor_cfg {
	uint8_t sensor_num;
	uint16_t data_addr;
} modbus_sensor_cfg;
//sensor_monitor_table_info

/* define modbus data address */
#define MODBUS_TEMP_BB_TMP75_ADDR 0x0101
#define MODBUS_TEMP_BPB_TMP75_ADDR 0x0102
#define MODBUS_POWER_RPU_ADDR 0x9999
#define RPU_Coolant_Flow_Rate_LPM 0x9000
#define RPU_Coolant_Outlet_Temp_C 0x9001
#define RPU_Coolant_Inlet_Temp_C 0x9002
#define RPU_Coolant_Outlet_Pressure_kPa 0x9003
#define RPU_Coolant_Inlet_Pressure_kPa 0x9004
#define RPU_PWR_W 0x9005
#define AALC_TOTAL_PWR_W 0x9006
#define RPU_INPUT_VOLT_V 0x9007
#define RPU_Air_Inlet_Temp_C 0x9008
#define RPU_Pump_PWM_TACH_PCT 0x9009
#define RPU_Pump1_TACH_RPM 0x900A
#define RPU_Pump2_TACH_RPM 0x900B
#define RPU_Pump3_TACH_RPM 0x900C
#define RPU_FAN1_STATUS 0x900E
#define RPU_FAN2_STATUS 0x900E
#define RPU_FAN1_TACH_RPM 0x900F
#define RPU_FAN2_TACH_RPM 0x9010
#define AALC_Cooling_Capacity_W 0x9011
#define RPU_Pump1_STATUS 0x9012
#define RPU_Pump2_STATUS 0x9013
#define RPU_Pump3_STATUS 0x9014
#define RPU_Reservoir_Status 0x9016
#define RPU_LED_Reservoir_Status 0x9017
#define RPU_LED_Leakage_Status 0x9017
#define RPU_LED_Fault_Status 0x9017
#define RPU_LED_Power_Status 0x9017
#define BB_TMP75_TEMP_C 0x9018
#define BPB_RPU_OUTLET_TEMP_C 0x9019
#define PDB_HDC1080DMBR_TEMP_C 0x901A
#define BB_HSC_P48V_TEMP_C 0x901B
#define BPB_HSC_P48V_TEMP_C 0x901C
#define PB_1_HDC1080DMBR_TEMP_C 0x901D
#define PB_2_HDC1080DMBR_TEMP_C 0x901E
#define PB_3_HDC1080DMBR_TEMP_C 0x901F
#define PB_1_HSC_P48V_TEMP_C 0x9020
#define PB_2_HSC_P48V_TEMP_C 0x9021
#define PB_3_HSC_P48V_TEMP_C 0x9022
#define PB_1_HSC_P48V_VIN_VOLT_V 0x9023
#define PB_2_HSC_P48V_VIN_VOLT_V 0x9024
#define PB_3_HSC_P48V_VIN_VOLT_V 0x9025
#define BB_HSC_P51V_VIN_VOLT_V 0x9026
#define BPB_HSC_P51V_VIN_VOLT_V 0x9027
#define BB_HSC_P51V_IOUT_CURR_A 0x9028
#define BPB_HSC_P51V_IOUT_CURR_A 0x9029
#define PB_1_HSC_P48V_IOUT_CURR_A 0x902A
#define PB_2_HSC_P48V_IOUT_CURR_A 0x902B
#define PB_3_HSC_P48V_IOUT_CURR_A 0x902C
#define BB_HSC_P51V_PIN_PWR_W 0x902D
#define BPB_HSC_P51V_PIN_PWR_W 0x902E
#define PB_1_HSC_P48V_PIN_PWR_W 0x902F
#define Pump_1_Running 0x9030
#define Pump_2_Running  0x9032
#define Pump_3_Running  0x9034
#define PB_2_HSC_P48V_PIN_PWR_W 0x9038
#define PB_3_HSC_P48V_PIN_PWR_W 0x9039
#define PB_1_FAN_1_TACH_RPM 0x903A
#define PB_1_FAN_2_TACH_RPM 0x903B
#define PB_2_FAN_1_TACH_RPM 0x903C
#define PB_2_FAN_2_TACH_RPM 0x903D
#define PB_3_FAN_1_TACH_RPM 0x903E
#define PB_3_FAN_2_TACH_RPM 0x903F
#define BPB_RACK_PRESSURE_3_P_KPA 0x9040
#define BPB_RACK_PRESSURE_4_P_KPA 0x9041
#define BPB_RACK_LEVEL_1 0x9042
#define BPB_RACK_LEVEL_2 0x9043
#define BPB_CDU_LEVEL_3 0x9044
#define MB_HUM_PCT_RH 0x9045
#define PDB_HUM_PCT_RH 0x9046
#define PB_1_HUM_PCT_RH 0x9047
#define PB_2_HUM_PCT_RH 0x9048
#define PB_3_HUM_PCT_RH 0x9049
#define HEX_FAN_PWM_TACH_PCT 0x9100
#define HEX_PWR_W 0x9101
#define HEX_INPUT_VOLT_V 0x9102
#define HEX_INPUT_CURRENT_V 0x9103
#define HEX_FAN1_TACH_RPM 0x9104
#define HEX_FAN2_TACH_RPM 0x9105
#define HEX_FAN3_TACH_RPM 0x9106
#define HEX_FAN4_TACH_RPM 0x9107
#define HEX_FAN5_TACH_RPM 0x9108
#define HEX_FAN6_TACH_RPM 0x9109
#define HEX_FAN7_TACH_RPM 0x910A
#define HEX_FAN8_TACH_RPM 0x910B
#define HEX_FAN9_TACH_RPM 0x910C
#define HEX_FAN10_TACH_RPM 0x910D
#define HEX_Air_Outlet1_Temp_C 0x910E
#define HEX_Air_Outlet2_Temp_C 0x910F
#define HEX_Air_Inlet1_Temp_C 0x9110
#define HEX_Air_Inlet2_Temp_C 0x9111
#define HEX_Water_Inlet_Temp_C 0x9112
#define HEX_Bladder_Level_Status 0x9113
#define SB_HEX_AIR_OUTLET_3_TEMP_C 0x9114
#define SB_HEX_AIR_OUTLET_4_TEMP_C 0x9115
#define FB_3_HEX_INLET_TEMP_C 0x9116
#define FB_4_HEX_INLET_TEMP_C 0x9117
#define FB_5_HEX_INLET_TEMP_C 0x9118
#define FB_6_HEX_INLET_TEMP_C 0x9119
#define FB_7_HEX_INLET_TEMP_C 0x911A
#define FB_8_HEX_INLET_TEMP_C 0x911B
#define FB_9_HEX_INLET_TEMP_C 0x911C
#define FB_10_HEX_INLET_TEMP_C 0x911D
#define FB_11_HEX_INLET_TEMP_C 0x911E
#define FB_12_HEX_INLET_TEMP_C 0x911F
#define FB_13_HEX_INLET_TEMP_C 0x9120
#define FB_14_HEX_INLET_TEMP_C 0x9121
#define FB_1_HSC_TEMP_C 0x9122
#define FB_2_HSC_TEMP_C 0x9123
#define FB_3_HSC_TEMP_C 0x9124
#define FB_4_HSC_TEMP_C 0x9125
#define FB_5_HSC_TEMP_C 0x9126
#define FB_6_HSC_TEMP_C 0x9127
#define FB_7_HSC_TEMP_C 0x9128
#define FB_8_HSC_TEMP_C 0x9129
#define FB_9_HSC_TEMP_C 0x912A
#define FB_10_HSC_TEMP_C 0x912B
#define FB_11_HSC_TEMP_C 0x912C
#define FB_12_HSC_TEMP_C 0x912D
#define FB_13_HSC_TEMP_C 0x912E
#define FB_14_HSC_TEMP_C 0x912F
#define FB_1_HSC_P48V_VIN_VOLT_V 0x9130
#define FB_2_HSC_P48V_VIN_VOLT_V 0x9131
#define FB_3_HSC_P48V_VIN_VOLT_V 0x9132
#define FB_4_HSC_P48V_VIN_VOLT_V 0x9133
#define FB_5_HSC_P48V_VIN_VOLT_V 0x9134
#define FB_6_HSC_P48V_VIN_VOLT_V 0x9135
#define FB_7_HSC_P48V_VIN_VOLT_V 0x9136
#define FB_8_HSC_P48V_VIN_VOLT_V 0x9137
#define FB_9_HSC_P48V_VIN_VOLT_V 0x9138
#define FB_10_HSC_P48V_VIN_VOLT_V 0x9139
#define FB_11_HSC_P48V_VIN_VOLT_V 0x913A
#define FB_12_HSC_P48V_VIN_VOLT_V 0x913B
#define FB_13_HSC_P48V_VIN_VOLT_V 0x913C
#define FB_14_HSC_P48V_VIN_VOLT_V 0x913D
#define FB_1_HSC_P48V_IOUT_CURR_A 0x913E
#define FB_2_HSC_P48V_IOUT_CURR_A 0x913F
#define FB_3_HSC_P48V_IOUT_CURR_A 0x9140
#define FB_4_HSC_P48V_IOUT_CURR_A 0x9141
#define FB_5_HSC_P48V_IOUT_CURR_A 0x9142
#define FB_6_HSC_P48V_IOUT_CURR_A 0x9143
#define FB_7_HSC_P48V_IOUT_CURR_A 0x9144
#define FB_8_HSC_P48V_IOUT_CURR_A 0x9145
#define FB_9_HSC_P48V_IOUT_CURR_A 0x9146
#define FB_10_HSC_P48V_IOUT_CURR_A 0x9147
#define FB_11_HSC_P48V_IOUT_CURR_A 0x9148
#define FB_12_HSC_P48V_IOUT_CURR_A 0x9149
#define FB_13_HSC_P48V_IOUT_CURR_A 0x914A
#define FB_14_HSC_P48V_IOUT_CURR_A 0x914B
#define FB_1_HSC_P48V_PIN_PWR_W 0x914C
#define FB_2_HSC_P48V_PIN_PWR_W 0x914D
#define FB_3_HSC_P48V_PIN_PWR_W 0x914E
#define FB_4_HSC_P48V_PIN_PWR_W 0x914F
#define FB_5_HSC_P48V_PIN_PWR_W 0x9150
#define FB_6_HSC_P48V_PIN_PWR_W 0x9151
#define FB_7_HSC_P48V_PIN_PWR_W 0x9152
#define FB_8_HSC_P48V_PIN_PWR_W 0x9153
#define FB_9_HSC_P48V_PIN_PWR_W 0x9154
#define FB_10_HSC_P48V_PIN_PWR_W 0x9155
#define FB_11_HSC_P48V_PIN_PWR_W 0x9156
#define FB_12_HSC_P48V_PIN_PWR_W 0x9157
#define FB_13_HSC_P48V_PIN_PWR_W 0x9158
#define FB_14_HSC_P48V_PIN_PWR_W 0x9159
#define FB_11_FAN_TACH_RPM 0x915A
#define FB_12_FAN_TACH_RPM 0x915B
#define FB_13_FAN_TACH_RPM 0x915C
#define FB_14_FAN_TACH_RPM 0x915D
#define SB_HEX_PRESSURE_1_P_KPA 0x915E
#define SB_HEX_PRESSURE_2_P_KPA 0x915F
#define FB_1_HUM_PCT_RH 0x9160
#define FB_2_HUM_PCT_RH 0x9161
#define FB_3_HUM_PCT_RH 0x9162
#define FB_4_HUM_PCT_RH 0x9163
#define FB_5_HUM_PCT_RH 0x9164
#define FB_6_HUM_PCT_RH 0x9165
#define FB_7_HUM_PCT_RH 0x9166
#define FB_8_HUM_PCT_RH 0x9167
#define FB_9_HUM_PCT_RH 0x9168
#define FB_10_HUM_PCT_RH 0x9169
#define FB_11_HUM_PCT_RH 0x916A
#define FB_12_HUM_PCT_RH 0x916B
#define FB_13_HUM_PCT_RH 0x916C
#define FB_14_HUM_PCT_RH 0x916D
#define LEAK_RPU_INT 0x9202
#define LEAK_RACK_FLOOR_GPO_and_Relay 0x9202

//FRU Info
#define FRU_FB_PART_ADDR 0x19C4
#define FRU_MFR_MODEL_ADDR 0x19CC
#define FRU_MFR_DATE_ADDR 0x19D4
#define FRU_MFR_SERIEL_ADDR 0x19D8
#define FRU_WORKORDER_ADDR 0x19E0
#define FRU_HW_REVISION_ADDR 0x19E4
#define FRU_FW_REVISION_ADDR 0x19E8
#define FRU_TOTAL_UP_TIME_ADDR 0x19EC
#define FRU_LAST_ON_TIME_ADDR 0x19EF
#define FRU_HMI_REVISION_ADDR 0x19F2
#define FRU_NOAH_ARK_CONFIG_ADDR 0x19F8
#define FRU_HEATEXCHANGER_CONTROLBOX_FPBN_ADDR 0x1A00
#define FRU_QUANTA_FB_PART_ADDR 0x1A0C

enum adm1272_write_enable_status{
	ADM1272_WRITE_ENABLE_SUCCESS,
	ADM1272_WRITE_ENABLE_FAIL,
};