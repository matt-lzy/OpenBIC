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

#include <stdint.h>

#define LED_TURN_OFF 0x00
#define LED_TURN_ON 0x01
#define LED_START_BLINK 0x02
#define LED_STOP_BLINK 0x03

enum LED_IDX_E {
	LED_IDX_E_POWER = 0,
	LED_IDX_E_FAULT,
	LED_IDX_E_LEAK,
	LED_IDX_E_COOLANT,
	LED_IDX_E_MAX,
};

// FM_LED_FP_4_EN //coolant led
// FM_LED_FP_3_EN //leak led
// FM_LED_FP_2_EN //fault led
// FM_LED_FP_1_EN //pwr led

enum LED_PUMP_FAULT_THRESHOLD_E {
	LED_FAULT_PB_1,
	LED_FAULT_PB_2,
	LED_FAULT_PB_3,
};

enum LED_HEX_FAN_FAULT_THRESHOLD_E {
	LED_FAULT_HEX_FAN_1,
	LED_FAULT_HEX_FAN_2,
	LED_FAULT_HEX_FAN_3,
	LED_FAULT_HEX_FAN_4,
	LED_FAULT_HEX_FAN_5,
	LED_FAULT_HEX_FAN_6,
	LED_FAULT_HEX_FAN_7,
	LED_FAULT_HEX_FAN_8,
	LED_FAULT_HEX_FAN_9,
	LED_FAULT_HEX_FAN_10,
	LED_FAULT_HEX_FAN_11,
	LED_FAULT_HEX_FAN_12,
	LED_FAULT_HEX_FAN_13,
	LED_FAULT_HEX_FAN_14,
};

enum LED_FAULT_THRESHOLD_E {
	LED_FAULT_PB_1_FAN_1,
	LED_FAULT_PB_1_FAN_2,
	LED_FAULT_PB_2_FAN_1,
	LED_FAULT_PB_2_FAN_2,
	LED_FAULT_PB_3_FAN_1,
	LED_FAULT_PB_3_FAN_2,
	LED_FAULT_HIGH_PRESS,
	LED_FAULT_LOW_LEVEL,
	LED_FAULT_HIGH_AIR_TEMP,
	LED_FAULT_HIGH_COOLANT_INLET_TEMP,
	LED_FAULT_HIGH_COOLANT_OUTLET_TEMP,
	LED_FAULT_FLOW_TRIGGER,
	LED_FAULT_THRESHOLD_E_MAX,
};

void led_set(uint8_t idx, uint8_t behaviour);
void led_ctrl(uint8_t idx, uint8_t ctrl);
uint8_t get_led_pin(uint8_t idx);
uint8_t get_led_status(uint8_t idx);
//void SSDLEDInit(void);
bool fault_led_control(void);