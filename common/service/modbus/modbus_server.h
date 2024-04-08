#include <modbus/modbus.h>


enum {
	MODBUS_READ_WRITE_REGISTER_SUCCESS,
	MODBUS_FUNCCODE_NOT_SUPPORT,
	MODBUS_VAL_NOT_IN_RANGE,
	MODBUS_ADDR_NOT_DEFINITION,
	MODBUS_READ_WRITE_REGISTER_FAIL,
};

int init_modbus_server(const char iface_name, struct modbus_iface_param server_param);