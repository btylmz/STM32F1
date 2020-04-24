#include "DS3231.h"

DS3231 rtc;
DS3231_days days;

uint8_t BCD2DEC(uint8_t data)
{
	return (data >> 4)*10 + (data & 0x0F);
}

uint8_t DEC2BCD(uint8_t data)
{
	return ((data/10) << 4) | (data % 10);
}

void DS3231_getTime()
{
	uint8_t buffer[7];
	I2C_readByte(hi2cX,DS3231_ADDRESS,DS3231_TIME_CAL,buffer,7);
	
	rtc.sec  = BCD2DEC(buffer[0]);
	rtc.min  = BCD2DEC(buffer[1]);
	rtc.hour = BCD2DEC(buffer[2]);
	rtc.dayofweek = BCD2DEC(buffer[3]);
	rtc.day   = BCD2DEC(buffer[4]);
	rtc.month = BCD2DEC(buffer[5]);
	rtc.year  = BCD2DEC(buffer[6]);
	
}

void DS3231_setTime(DS3231 dt)
{
	uint8_t buffer[7];
	
	buffer[0] = DEC2BCD(dt.sec);
	buffer[1] = DEC2BCD(dt.min);
	buffer[2] = DEC2BCD(dt.hour);
	buffer[3] = DEC2BCD(dt.dayofweek);
	buffer[4] = DEC2BCD(dt.day);
	buffer[5] = DEC2BCD(dt.month);
	buffer[6] = DEC2BCD(dt.year);
	
	I2C_writeByte(hi2cX,DS3231_ADDRESS,DS3231_TIME_CAL,buffer,7);
		
}

