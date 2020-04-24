#include "EEPROM.h"
#include "string.h"

void EEPROM_writeByte(I2C_HandleTypeDef hi2cX,uint8_t address,uint8_t position,uint8_t * wdata,uint8_t size)
{
	if(size <= 16)
	{
		uint8_t buffer[size];
	
		for(int i = 0; i < size; i++)
		{
			buffer[i] = wdata[i];
		}
	
		I2C_writeByte(hi2cX,address,position,buffer,size);
	}
	
}

void EEPROM_readByte(I2C_HandleTypeDef hi2cX,uint8_t address,uint8_t position,uint8_t * rdata,uint8_t size)
{
	if(size <= 16)
	{
		uint8_t buffer[size];
	
		I2C_readByte(hi2cX,address,position,buffer,size);
	
		for(int i = 0; i < size; i++)
		{
			rdata[i] = buffer[i];
		}
		
	}
}

void EEPROM_writeNum(I2C_HandleTypeDef hi2cX,uint8_t address,uint8_t position,int16_t  wdata)
{
	uint8_t buffer[2];
	
	buffer[0] = wdata >> 8;
	buffer[1] = wdata & 0xFF;
	
	I2C_writeByte(hi2cX,address,position,buffer,2);
}

uint16_t EEPROM_readNum(I2C_HandleTypeDef hi2cX,uint8_t address,uint8_t position)
{
	uint8_t buffer[2];
	uint16_t data;
	
	I2C_readByte(hi2cX,address,position,&buffer[0],1);
	I2C_readByte(hi2cX,address,position + 1,&buffer[1],1);
	
	data = ((int16_t)buffer[0] << 8) | (buffer[1]);
	return data;
	
}

void EEPROM_reset(I2C_HandleTypeDef hi2cX)
{
	uint8_t buffer = 0;
	
	for(int i = 0; i < 8; i++)
	{
		for(int u = 0; u < 16; u++)
		{
			I2C_writeByte(hi2cX,EEPROM_BLOCK_0 + i,u,&buffer,1);
		}
		
	}
	
}
