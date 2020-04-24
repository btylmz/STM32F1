#include "BMP180.h"
#include "math.h"

BMP180_calib_data BMP180_calib;
uint8_t _sampling;
uint8_t BMP180_conversion_time;

int16_t LastTemperatureData,LastTemperatureTime,AcceptableTemperatureLatencyForPressure = 1000;

void BMP180_Reset()
{
	uint8_t buffer = 0xB6;
	I2C_writeByte(BMP180_hi2cX,BMP180_Address,BMP180_reset,&buffer,1);
	HAL_Delay(100);
}

void BMP180_Init(enum BMP180_sensor_sampling sampling)
{
	uint8_t buffer[2];
	
	switch(sampling)
	{
		case(0x00):
			BMP180_conversion_time = 5;
			break;
			
		case(0x01):
			BMP180_conversion_time = 8;
			break;
		
		case(0x10):
			BMP180_conversion_time = 14;
			break;		
		
		case(0x11):
			BMP180_conversion_time = 26;
			break;
	}
	
	I2C_readByte(BMP180_hi2cX,BMP180_Address,BMP180_AC1_MSB,buffer,2);
	BMP180_calib.AC1 = (buffer[0] << 8) | buffer[1];
	buffer[0] = 0;
	buffer[1] = 0;
	I2C_readByte(BMP180_hi2cX,BMP180_Address,BMP180_AC2_MSB,buffer,2);
	BMP180_calib.AC2 = (buffer[0] << 8) | buffer[1];	
	buffer[0] = 0;
	buffer[1] = 0;
	I2C_readByte(BMP180_hi2cX,BMP180_Address,BMP180_AC3_MSB,buffer,2);
	BMP180_calib.AC3 = (buffer[0] << 8) | buffer[1];
	buffer[0] = 0;
	buffer[1] = 0;
	I2C_readByte(BMP180_hi2cX,BMP180_Address,BMP180_AC4_MSB,buffer,2);
	BMP180_calib.AC4 = (buffer[0] << 8) | buffer[1];
	buffer[0] = 0;
	buffer[1] = 0;
	I2C_readByte(BMP180_hi2cX,BMP180_Address,BMP180_AC5_MSB,buffer,2);
	BMP180_calib.AC5 = (buffer[0] << 8) | buffer[1];
	buffer[0] = 0;
	buffer[1] = 0;
	I2C_readByte(BMP180_hi2cX,BMP180_Address,BMP180_AC6_MSB,buffer,2);
	BMP180_calib.AC6 = (buffer[0] << 8) | buffer[1];
	buffer[0] = 0;
	buffer[1] = 0;
	I2C_readByte(BMP180_hi2cX,BMP180_Address,BMP180_B1_MSB,buffer,2);
	BMP180_calib.B1 = (buffer[0] << 8) | buffer[1];
	buffer[0] = 0;
	buffer[1] = 0;
	I2C_readByte(BMP180_hi2cX,BMP180_Address,BMP180_B2_MSB,buffer,2);
	BMP180_calib.B2 = (buffer[0] << 8) | buffer[1];
	buffer[0] = 0;
	buffer[1] = 0;
	I2C_readByte(BMP180_hi2cX,BMP180_Address,BMP180_MB_MSB,buffer,2);
	BMP180_calib.MB = (buffer[0] << 8) | buffer[1];
	buffer[0] = 0;
	buffer[1] = 0;
	I2C_readByte(BMP180_hi2cX,BMP180_Address,BMP180_MC_MSB,buffer,2);
	BMP180_calib.MC = (buffer[0] << 8) | buffer[1];
	buffer[0] = 0;
	buffer[1] = 0;
	I2C_readByte(BMP180_hi2cX,BMP180_Address,BMP180_MD_MSB,buffer,2);
	BMP180_calib.MD = (buffer[0] << 8) | buffer[1];
	buffer[0] = 0;
	buffer[1] = 0;
	
	_sampling = sampling;
	
}

long BMP180_getUncompensatedTemperature(void)
{
	uint8_t buffer[3];
	long UT;
	
	buffer[0] = BMP180_measure_Temperature;
	I2C_writeByte(BMP180_hi2cX,BMP180_Address,BMP180_ctrl_meas,buffer,1);
	
	HAL_Delay(BMP180_conversion_time);
	
	I2C_readByte(BMP180_hi2cX,BMP180_Address,BMP180_out_msb,buffer,3);
	
	UT = ((long)buffer[0] << 8) | (long)buffer[1];
	return UT;
	
}

long BMP180_getUncompensatedPressure(void)
{
	uint8_t buffer[3];
	long UP;
	
	buffer[0] = 0x34 + (_sampling << 6);
	I2C_writeByte(BMP180_hi2cX,BMP180_Address,BMP180_ctrl_meas,buffer,1);
	HAL_Delay(BMP180_conversion_time);
	
	I2C_readByte(BMP180_hi2cX,BMP180_Address,BMP180_out_msb,buffer,3);
	
	UP = (((long)buffer[0] << 16) | ((long)buffer[1] << 8) | (long)buffer[0] ) >> (8 - _sampling);
	return UP;
	
}

void BMP180_readTemperature(double * temperature)
{
	uint8_t buffer[3];
	long UT;
	long temp;
	
	UT = BMP180_getUncompensatedTemperature();
	
	long x1;
	long x2;
	
	x1 = ((UT - (long)BMP180_calib.AC6) * ((long)BMP180_calib.AC5) >> 15);
  	x2 = ((long)BMP180_calib.MC << 11) / (x1 + BMP180_calib.MD);
	
	long param_B5 = x1 + x2;
	
	temp = ((param_B5 + 8) >> 4);
	
	*temperature = temp;
	*temperature /= 10.0;
	
  	LastTemperatureData = param_B5;
  	LastTemperatureTime = HAL_GetTick();
		
	 
}
void BMP180_readPressure(double * pressure)
{
	uint8_t buffer[3];	double temp;
	long UP;
	
	int msSinceLastTempReading = HAL_GetTick() - LastTemperatureTime;
	
	// Check to see if we have old temperature data.
    if (msSinceLastTempReading > AcceptableTemperatureLatencyForPressure)
        BMP180_readTemperature(&temp); // Refresh the temperature.
	
		UP = BMP180_getUncompensatedPressure();
	
	// Algorithm taken from BMP180 datasheet.
    long b6 = LastTemperatureData - 4000;
    long x1 = (BMP180_calib.B2 * ((b6*b6) >> 12)) >> 11;
    long x2 = (BMP180_calib.AC2 * b6) >> 11;
    long x3 = x1 + x2;
    long b3 = (((BMP180_calib.AC1 * 4 + x3) << _sampling) + 2) >> 2;
    
    x1 = (BMP180_calib.AC3 * b6) >> 13;
    x2 = ((BMP180_calib.B1) * ((b6*b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    long b4 = (BMP180_calib.AC4 * (x3 + 32768)) >> 15;
    unsigned long b7 = ((UP) - b3) * (50000 >> _sampling);
    long p;
    if(b7 < 0x80000000)
    {
    	p = ((b7 * 2) / b4);
	}
	else
	{
		p = ((b7/b4) * 2);
		
	}
	
		x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);
    
    *pressure = (double)p;
	
}
void BMP180_readData(double * temperature,double * pressure)
{
	BMP180_readTemperature(temperature);
	BMP180_readPressure(pressure);
}

double BMP180_readAltitude(double pressure,float sealevel_hPa)
{
	double altitude;
	pressure /= 100;
	altitude = 44330 * (1-pow(pressure/sealevel_hPa,1/5.255));
	return altitude;
}
