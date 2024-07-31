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

#define TWO_BYTES_SENSOR_STATUS 0xFF

enum AALC_SENSOR_STATUS_E {
	// RPU FAN STATUS
	RPU_FAN_STATUS,
	// RPU PUMP STATUS
	RPU_PUMP1_STATUS,
	RPU_PUMP2_STATUS,
	RPU_PUMP3_STATUS,
	// RPU LED STATUS
	RPU_RESERVOIR_STATUS,
	RPU_LED_STATUS,
	// RPU STATUS
	RPU_PUMP_STATUS,
	RPU_INTERNAL_FAN_STATUS,
	// PUMP FAN STATUS 0xa080
	PUMP_FAN_STATUS,
	// HEX STATUS
	HEX_BLADDER_LEVEL_STATUS,
	HEX_EXTERNAL_Y_FILTER_PRESSURE,
	// AALC Sensor Alarm 0x9200
	AALC_SENSOR_ALARM,
	// AALC Sensor Alarm 0xa202
	HEX_AIR_THERMOMETER_STATUS,
	// AALC Status Alarm 0x9201
	AALC_STATUS_ALARM,
	// Leakage 0x9202
	SB_TTV_COOLANT_LEAKAGE,
	// HEX Alarm 0x9203, 0xa201
	HEX_FAN_ALARM_1,
	HEX_FAN_ALARM_2,
	// Leakage Black Box 0x19a1~0x19ac, 0xa300~0xa302
	STICKY_ITRACK_CHASSIS0_LEAKAGE,
	STICKY_ITRACK_CHASSIS1_LEAKAGE,
	STICKY_ITRACK_CHASSIS2_LEAKAGE,
	STICKY_ITRACK_CHASSIS3_LEAKAGE,
	STICKY_RPU_INTERNAL_LEAKAGE_ABNORMAL,
	STICKY_RPU_EXTERNAL_LEAKAGE_ABNORMAL,
	STICKY_RPU_OPT_EXTERNAL_LEAKAGE1_ABNORMAL,
	STICKY_RPU_OPT_EXTERNAL_LEAKAGE2_ABNORMAL,
	STICKY_HEX_RACK_PAN_LEAKAGE,
	STICKY_HEX_RACK_FLOOR_LEAKAGE,
	STICKY_HEX_RACK_PAN_LEAKAGE_RELAY,
	STICKY_HEX_RACK_FLOOR_LEAKAGE_RELAY,
	LED_FAULT,
	MAX_NUM_OF_AALC_STATUS
};

enum AALC_SENSOR_ALARM_E {
	RPU_COOLANT_INLET_THERMOMETER_STATUS = 0,
	RPU_COOLANT_OUTLET_THERMOMETER_STATUS = 1,
	RPU_COOLANT_INLET_GAUGE_STATUS = 2,
	RPU_COOLANT_OUTLET_GAUGE_STATUS = 3,
	RPU_FLOW_METER_STATUS = 4,
	HEX_AIR_OUTLET1_THERMOMETER_STATUS = 5,
	HEX_AIR_OUTLET2_THERMOMETER_STATUS = 6,
	HEX_AIR_INLET1_THERMOMETER_STATUS = 7,
	HEX_AIR_INLET2_THERMOMETER_STATUS = 8,
	HX_WATER_INLET_THERMOMETER_STATUS = 9,
	RPU_AIR_INLET_THERMOMETER_STATUS = 10, // not support
	RPU_HSC_STATUS = 11,
	RPU_MIDDLE_WATER_LEVEL_GAUGE = 12, // not support
	RPU_LOW_WATER_LEVEL_GAUGE = 13, // not support
	STATIC_PRESSURE_STATUS = 14,
	BLADDER_LEVEL_SENSOR_STATUS = 15,
};

enum RPU_RESERVOIR_STATUS_E {
	LEVEL2_STATUS = 0,
	LEVEL1_STATUS = 1,
};

enum HEX_AIR_THERMOMETER_STATUS_E {
	HEX_AIR_OUTLET3_THERMOMETER_STATUS = 0,
	HEX_AIR_OUTLET4_THERMOMETER_STATUS = 1,
	HEX_AIR_OUTLET5_THERMOMETER_STATUS = 2,
	HEX_AIR_OUTLET6_THERMOMETER_STATUS = 3,
	HEX_AIR_OUTLET7_THERMOMETER_STATUS = 4,
	HEX_AIR_OUTLET8_THERMOMETER_STATUS = 5,
	HEX_AIR_OUTLET9_THERMOMETER_STATUS = 6,
	HEX_AIR_OUTLET10_THERMOMETER_STATUS = 7,
	HEX_AIR_OUTLET11_THERMOMETER_STATUS = 8,
	HEX_AIR_OUTLET12_THERMOMETER_STATUS = 9,
	HEX_AIR_OUTLET13_THERMOMETER_STATUS = 10,
	HEX_AIR_OUTLET14_THERMOMETER_STATUS = 11,
	HEX_AIR_INLET3_THERMOMETER_STATUS = 12,
	HEX_AIR_INLET4_THERMOMETER_STATUS = 13,
};

enum LED_FAULT_E {
	LED_FAULT_PUMP = 0,
	LED_FAULT_RPU_FAN = 1,
	LED_FAULT_HEX_FAN = 2,
	LED_FAULT_HIGH_PRESS = 3,
	LED_FAULT_LOW_LEVEL = 4,
	LED_FAULT_HIGH_AIR_TEMP = 5,
	LED_FAULT_HIGH_COOLANT_TEMP = 6,
	LED_FAULT_FLOW_TRIGGER = 7,
	LED_FAULT_IT_LEAK = 8,
	LED_FAULT_CDU_LEAKAGE = 9,
	LED_FAULT_RACK_LEAKAGE = 10,
};

void set_threshold_poll_enable_flag(bool flag);
bool get_threshold_poll_enable_flag();
void threshold_poll_init();
void fan_pump_pwrgd();
uint16_t get_sensor_status(uint8_t sensor_status_num, uint8_t bit);
bool set_sensor_status(uint8_t sensor_status_num, uint8_t bit, uint16_t val);
uint16_t read_sensor_status(uint8_t sensor_status_num);