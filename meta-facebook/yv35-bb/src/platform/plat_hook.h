#ifndef PLAT_HOOK_H
#define PLAT_HOOK_H

typedef struct _ltc4282_pre_proc_arg {
	char *vsource_status;
} ltc4282_pre_proc_arg;

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
extern adc_asd_init_arg adc_asd_init_args[];

extern adm1278_init_arg adm1278_init_args[];

extern ltc4282_init_arg ltc4282_init_args[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
extern ltc4282_pre_proc_arg ltc4282_pre_read_args[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
bool pre_ltc4282_read(uint8_t sensor_num, void *args);

#endif
