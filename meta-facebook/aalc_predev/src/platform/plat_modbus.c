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
#include "hal_gpio.h"
#include "plat_gpio.h"

LOG_MODULE_REGISTER(plat_modbus);

struct k_thread modbus_server_thread;
K_KERNEL_STACK_MEMBER(modbus_server_thread_stack, MODBUS_SERVER_THREAD_SIZE);

static char server_iface_name[] = "MODBUS0";
//{DT_PROP(DT_INST(1, zephyr_modbus_serial), label)};
//static uint16_t i2c_data_wr[125];
static uint16_t regs_wr[MODBUS_FC16_REGS_QUANTITY];
static uint16_t regs_rd[MODBUS_FC3_REGS_QUANTITY];
static uint8_t cb_num;
static uint16_t cb_addr[2];

modbus_sensor_cfg plat_modbus_sensors[] = {
	/* sensor number,  modbus data addr  */
	{ SENSOR_NUM_BB_TMP75_TEMP_C, MODBUS_TEMP_BB_TMP75_ADDR },
	{ SENSOR_NUM_BPB_RPU_OUTLET_TEMP_C, MODBUS_TEMP_BPB_TMP75_ADDR },
};

static int holding_reg_rd(uint16_t addr, uint16_t *reg, uint16_t reg_qty)
{
	if (reg_qty >= MODBUS_FC3_REGS_QUANTITY)
		return -ENOTSUP;

	if (cb_num > 0 && (addr > cb_addr[0] && addr <= cb_addr[1])) {
		*reg = regs_rd[reg_qty - cb_num];
		cb_num--;
		return MODBUS_READ_WRITE_REGISTER_SUCCESS;
	}

	cb_num = 0;
	memset(&cb_addr, 0, sizeof(cb_addr));
	memset(&regs_rd, 0, sizeof(regs_rd));

	if (addr < 0x1000) { //GPIO & i2c Read test
		if (addr < 0x0100) {
			if (addr < plat_gpio_cfg_size()) { //GPIO number
				if (reg_qty != 1)
					return MODBUS_NOT_IN_REGISTER_VAL_RANGE;

				*reg = (uint16_t)gpio_get((uint8_t)addr);
				return MODBUS_READ_WRITE_REGISTER_SUCCESS;
			} else {
				return MODBUS_ADDR_NOT_DEFINITION;
			}
		} else { //I2C BUS & Address(addr= 0x0YZZ, Y:bus ZZ:Address)
			I2C_MSG msg;
			msg.bus = ((uint8_t)(addr >> 8)) - 1; //base 1 -> 0 base
			msg.target_addr = (uint8_t)(addr & 0x00ff);
			msg.rx_len = (uint8_t)reg_qty * 2;
			msg.tx_len = 0;

			int ret = i2c_master_read(&msg, 3);
			if (ret != 0)
				return MODBUS_READ_WRITE_REGISTER_FAIL;

			for (uint8_t i = 0; i < msg.rx_len; i++) {
				regs_rd[i] =
					(uint16_t)((msg.data[2 * i] << 8) + msg.data[2 * i + 1]);
			}

			cb_num = reg_qty - 1;
			*reg = regs_rd[0];
			cb_addr[0] = addr;
			cb_addr[1] = addr + cb_num;
			return MODBUS_READ_WRITE_REGISTER_SUCCESS;
		}
	} else if (addr < 0x9200) { // sensor addr is less than 0x9200
		int reading = 0;
		if ((reg_qty * 2) != sizeof(reading))
			return MODBUS_NOT_IN_REGISTER_VAL_RANGE;

		for (uint8_t index = 0; index < sensor_config_count; index++) {
			if (plat_modbus_sensors[index].data_addr == addr) {
				uint8_t status =
					get_sensor_reading(sensor_config, sensor_config_count,
							   plat_modbus_sensors[index].sensor_num,
							   &reading, GET_FROM_CACHE);

				if (status ==
				    SENSOR_READ_SUCCESS) { //reading type is int(4Bytes = 2 registers)
					regs_rd[1] = reading & 0x0000FFFF;
					regs_rd[0] = (reading >> 8) >> 8;
					cb_num = reg_qty - 1;
					*reg = regs_rd[0];
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
				*reg = regs_rd[0];
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
	if (reg_qty >= MODBUS_FC16_REGS_QUANTITY)
		return -ENOTSUP;

	if (reg_qty > 0) {
		if (cb_num == 0) {
			cb_num = reg_qty;
			cb_addr[1] = addr + cb_num - 1;
			cb_addr[0] = addr;
		}

		if (cb_num >= 0 && (addr >= cb_addr[0] && addr <= cb_addr[1])) {
			regs_wr[reg_qty - cb_num] = reg;
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
	if (cb_addr[0] < 0x1000) { //GPIO & i2c Read test
		if (cb_addr[0] < 0x0100) {
			if (cb_addr[0] < plat_gpio_cfg_size()) { //GPIO number
				if ((reg_qty != 1) || (regs_wr[0] > 1))
					return MODBUS_NOT_IN_REGISTER_VAL_RANGE;

				gpio_set((uint8_t)cb_addr[0], (uint8_t)regs_wr[0]);
				memset(&cb_addr, 0, sizeof(cb_addr));
				memset(&regs_wr, 0, sizeof(regs_wr));

				return MODBUS_READ_WRITE_REGISTER_SUCCESS;
			} else {
				memset(&cb_addr, 0, sizeof(cb_addr));
				memset(&regs_wr, 0, sizeof(regs_wr));
				return MODBUS_ADDR_NOT_DEFINITION;
			}
		} else { //I2C BUS & Address(addr= 0x0YZZ, Y:bus ZZ:Address)
			uint8_t i2c_retry = 3;
			I2C_MSG msg;
			msg.bus = ((uint8_t)(addr >> 8)) - 1; //1 base -> 0base
			msg.target_addr = (uint8_t)(cb_addr[0] & 0x00ff);
			msg.tx_len = (uint8_t)(reg_qty * 2);

			for (uint8_t i = 0; i < reg_qty; i++) {
				msg.data[2 * i] = (uint8_t)(regs_wr[2 * i] << 8);
				msg.data[2 * i + 1] = (uint8_t)(regs_wr[2 * i] && 0X00FF);
			}

			//memcpy(&i2c_data_wr[0], &regs_wr[0], msg.tx_len);

			memset(&cb_addr, 0, sizeof(cb_addr));
			memset(&regs_wr, 0, sizeof(regs_wr));

			if (i2c_master_write(&msg, i2c_retry))
				return MODBUS_READ_WRITE_REGISTER_FAIL;
			else
				return MODBUS_READ_WRITE_REGISTER_SUCCESS;
		}
	} else {
		uint8_t status = 0;
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
			if (status == FRU_WRITE_SUCCESS) {
				return MODBUS_READ_WRITE_REGISTER_SUCCESS;
			} else {
				return MODBUS_READ_WRITE_REGISTER_FAIL;
			}
		default:
			memset(&cb_addr, 0, sizeof(cb_addr));
			memset(&regs_wr, 0, sizeof(regs_wr));
			LOG_ERR("Read Modbus Sensor failed");
			return MODBUS_ADDR_NOT_DEFINITION;
		}
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
