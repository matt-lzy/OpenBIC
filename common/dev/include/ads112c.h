#ifndef ADS112C_H
#define ADS112C_H

enum ADS112C_REG0_CONFIG {
	//Input multiplexer configuration(Bit 7:4)
	ADS112C_REG0_INPUT_AIN0AIN1 = 0x00, //AINP = AIN0, AINN = AIN1(default)
	ADS112C_REG0_INPUT_AIN0AIN2 = 0x10, //AINP = AIN0, AINN = AIN2
	ADS112C_REG0_INPUT_AIN0AIN3 = 0x20, //AINP = AIN0, AINN = AIN3
	ADS112C_REG0_INPUT_AIN1AIN0 = 0x30, //AINP = AIN1, AINN = AIN0
	ADS112C_REG0_INPUT_AIN1AIN2 = 0x40, //AINP = AIN1, AINN = AIN2
	ADS112C_REG0_INPUT_AIN1AIN3 = 0x50, //AINP = AIN1, AINN = AIN3
	ADS112C_REG0_INPUT_AIN2AIN3 = 0x60, //AINP = AIN2, AINN = AIN3
	ADS112C_REG0_INPUT_AIN3AIN2 = 0x70, //AINP = AIN3, AINN = AIN2			
	ADS112C_REG0_INPUT_AIN0AVSS = 0x80, //AINP = AIN0, AINN = AVSS
	ADS112C_REG0_INPUT_AIN1AVSS = 0x90, //AINP = AIN1, AINN = AVSS
	ADS112C_REG0_INPUT_AIN2AVSS = 0xA0, //AINP = AIN2, AINN = AVSS
	ADS112C_REG0_INPUT_AIN3AVSS = 0xB0, //AINP = AIN3, AINN = AVSS
	//Gain configuration(Bit 3:1)
	ADS112C_REG0_GAIN1 = 0x00,		//Gain = 1 (default)
	ADS112C_REG0_GAIN2 = 0x02,
	ADS112C_REG0_GAIN4 = 0x04,
	ADS112C_REG0_GAIN8 = 0x06,
	ADS112C_REG0_GAIN16 = 0x08,
	ADS112C_REG0_GAIN32 = 0x0A,
	ADS112C_REG0_GAIN64 = 0x0C,
	ADS112C_REG0_GAIN128 = 0x0E,			
    //Disables and bypasses the internal low-noise PGA (Bit 0)
	ADS112C_REG0_PGA_ENABLE = 0x00,
	ADS112C_REG0_PGA_DISABLE = 0x01,	
};

enum ADS112C_REG1_CONFIG {
	//Conversion mode. (Bit 3)
	ADS112C_REG1_SINGLEMODE = 0x00,
	ADS112C_REG1_CONTINUEMODE = 0x08,
	//Voltage reference selection. (Bit 2:1)
	ADS112C_REG1_INTERNALV = 0x00,
	ADS112C_REG1_EXTERNALV = 0x02,	
};

// Command Byte to control device
#define CMD_RESET 0x06 //000 011x(06h)
#define CMD_START_SYNC 0x08 //000 100x(08h)
#define CMD_POWERDOWN 0x02 //000 001x(08h)
#define CMD_RDATA 0x10 //001 000x(10h)
#define CMD_RREG 0x20 //010 000x(20h)
#define CMD_WREG 0x40 //100 000x(40h)

// Configuration Registers offset
#define CFG_REG_OFFSET0 0x00
#define CFG_REG_OFFSET1 0x04
#define CFG_REG_OFFSET2 0x08
#define CFG_REG_OFFSET3 0x0C

typedef struct _ads112c_init_arg {
	uint8_t reg0_input;
	uint8_t reg0_gain;
	uint8_t reg0_pga;
	uint8_t reg1_conversion;
	uint8_t reg1_vol_refer;
} ads112c_init_arg;

#endif //ADS112C_H