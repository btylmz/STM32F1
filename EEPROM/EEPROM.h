// This library is written for 24LC16B EEPROM


#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "i2c.h"

// To use this library you should add the functions I2C_writeByte and I2C_readByte to the i2c.c 

#define EEPROM_BLOCK_0 0x50
#define EEPROM_BLOCK_1 0x51
#define EEPROM_BLOCK_2 0x52
#define EEPROM_BLOCK_3 0x53
#define EEPROM_BLOCK_4 0x54
#define EEPROM_BLOCK_5 0x55
#define EEPROM_BLOCK_6 0x56
#define EEPROM_BLOCK_7 0x57

void EEPROM_writeByte(I2C_HandleTypeDef hi2cX,uint8_t address,uint8_t position,uint8_t * wdata,uint8_t size);
void EEPROM_readByte(I2C_HandleTypeDef hi2cX,uint8_t address,uint8_t position,uint8_t * rdata,uint8_t size);
void EEPROM_writeNum(I2C_HandleTypeDef hi2cX,uint8_t address,uint8_t position,int16_t  wdata);
uint16_t EEPROM_readNum(I2C_HandleTypeDef hi2cX,uint8_t address,uint8_t position);
void EEPROM_reset(I2C_HandleTypeDef hi2cX);




