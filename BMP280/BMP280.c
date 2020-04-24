#include "BMP280.h"
#include "math.h"

BMP280_calib_data dig;
 float adc_P,adc_T;
 
 
uint8_t BMP280_getID()
{
	uint8_t address = 0;
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_ID,&address,1);
	return address;
	
}
uint8_t BMP280_getStatus()
{
	uint8_t mask_im_update = 0x01;
	uint8_t mask_measuring = 0x08;
	uint8_t  rbuffer;
	
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_STATUS,&rbuffer,1);
	
	if((rbuffer & mask_im_update) == 0x01)
	{
		return BMP280_im_update;
	}
	else if((rbuffer & mask_measuring) == 0x08)
	{
		return BMP280_measuring;
	}
	else
	{
		return 0;
	}
}

void BMP280_Init(enum BMP280_sensor_mode mode,enum BMP280_sensor_sampling tempSampling,enum BMP280_sensor_sampling pressSampling,enum BMP280_sensor_filter filter,enum BMP280_standby_duration duration)
{
	uint8_t I2C_wbuffer[3];
	uint8_t I2C_rbuffer[2];
	
	// Reset the device
	I2C_wbuffer[0] = 0xB6;
	I2C_writeByte(hi2cX,BMP280_ADDRESS,BMP280_RESET,I2C_wbuffer,1);
	
	// Set the sampling rates and mode
	I2C_wbuffer[0] = (tempSampling << 5) | (pressSampling << 2) | (mode);
	I2C_writeByte(hi2cX,BMP280_ADDRESS,BMP280_CTRL_MEAS,I2C_wbuffer,1);
	
	// Set the standby duration and filter
	I2C_wbuffer[0] = ((duration << 5) | (filter << 2)) & 0xFC;
	I2C_writeByte(hi2cX,BMP280_ADDRESS,BMP280_CONFIG,I2C_wbuffer,1);	
	
	// Read the calibration data and store it.
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digT1_MSB,&I2C_rbuffer[1],1);
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digT1_LSB,&I2C_rbuffer[0],1);
	dig.dig_T1 = (I2C_rbuffer[1] << 8) | (I2C_rbuffer[0]);
	// Make sure buffer is empty
	I2C_rbuffer[0] &= 0x00;
	I2C_rbuffer[1] &= 0x00;

	// Read the calibration data and store it.
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digT2_MSB,&I2C_rbuffer[1],1);
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digT2_LSB,&I2C_rbuffer[0],1);
	dig.dig_T2 = (I2C_rbuffer[1] << 8) | (I2C_rbuffer[0]);
	// Make sure buffer is empty
	I2C_rbuffer[0] &= 0x00;
	I2C_rbuffer[1] &= 0x00;
	
	// Read the calibration data and store it.
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digT3_MSB,&I2C_rbuffer[1],1);
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digT3_LSB,&I2C_rbuffer[0],1);
	dig.dig_T3 = (I2C_rbuffer[1] << 8) | (I2C_rbuffer[0]);
	// Make sure buffer is empty
	I2C_rbuffer[0] &= 0x00;
	I2C_rbuffer[1] &= 0x00;	

	// Read the calibration data and store it.
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digP1_MSB,&I2C_rbuffer[1],1);
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digP1_LSB,&I2C_rbuffer[0],1);
	dig.dig_P1 = (I2C_rbuffer[1] << 8) | (I2C_rbuffer[0]);
	// Make sure buffer is empty
	I2C_rbuffer[0] &= 0x00;
	I2C_rbuffer[1] &= 0x00;	
	
	// Read the calibration data and store it.
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digP2_MSB,&I2C_rbuffer[1],1);
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digP2_LSB,&I2C_rbuffer[0],1);
	dig.dig_P2 = (I2C_rbuffer[1] << 8) | (I2C_rbuffer[0]);
	// Make sure buffer is empty
	I2C_rbuffer[0] &= 0x00;
	I2C_rbuffer[1] &= 0x00;	
	
	// Read the calibration data and store it.
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digP3_MSB,&I2C_rbuffer[1],1);
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digP3_LSB,&I2C_rbuffer[0],1);
	dig.dig_P3 = (I2C_rbuffer[1] << 8) | (I2C_rbuffer[0]);
	// Make sure buffer is empty
	I2C_rbuffer[0] &= 0x00;
	I2C_rbuffer[1] &= 0x00;	
	
	// Read the calibration data and store it.
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digP4_MSB,&I2C_rbuffer[1],1);
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digP4_LSB,&I2C_rbuffer[0],1);
	dig.dig_P4 = (I2C_rbuffer[1] << 8) | (I2C_rbuffer[0]);
	// Make sure buffer is empty
	I2C_rbuffer[0] &= 0x00;
	I2C_rbuffer[1] &= 0x00;	
	
	// Read the calibration data and store it.
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digP5_MSB,&I2C_rbuffer[1],1);
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digP5_LSB,&I2C_rbuffer[0],1);
	dig.dig_P5 = (I2C_rbuffer[1] << 8) | (I2C_rbuffer[0]);
	// Make sure buffer is empty
	I2C_rbuffer[0] &= 0x00;
	I2C_rbuffer[1] &= 0x00;	
	
	// Read the calibration data and store it.
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digP6_MSB,&I2C_rbuffer[1],1);
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digP6_LSB,&I2C_rbuffer[0],1);
	dig.dig_P6 = (I2C_rbuffer[1] << 8) | (I2C_rbuffer[0]);
	// Make sure buffer is empty
	I2C_rbuffer[0] &= 0x00;
	I2C_rbuffer[1] &= 0x00;	
	
	// Read the calibration data and store it.
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digP7_MSB,&I2C_rbuffer[1],1);
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digP7_LSB,&I2C_rbuffer[0],1);
	dig.dig_P7 = (I2C_rbuffer[1] << 8) | (I2C_rbuffer[0]);
	// Make sure buffer is empty
	I2C_rbuffer[0] &= 0x00;
	I2C_rbuffer[1] &= 0x00;	
	
	// Read the calibration data and store it.
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digP8_MSB,&I2C_rbuffer[1],1);
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digP8_LSB,&I2C_rbuffer[0],1);
	dig.dig_P8 = (I2C_rbuffer[1] << 8) | (I2C_rbuffer[0]);
	// Make sure buffer is empty
	I2C_rbuffer[0] &= 0x00;
	I2C_rbuffer[1] &= 0x00;	
	
	// Read the calibration data and store it.
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digP9_MSB,&I2C_rbuffer[1],1);
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_digP9_LSB,&I2C_rbuffer[0],1);
	dig.dig_P9 = (I2C_rbuffer[1] << 8) | (I2C_rbuffer[0]);
	// Make sure buffer is empty
	I2C_rbuffer[0] &= 0x00;
	I2C_rbuffer[1] &= 0x00;	

	
}

void BMP280Temp_Pres_Calc(double* temperature, double* pressure)
{
	uint8_t I2C_rbuffer[3];
	
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_PRESS_MSB,I2C_rbuffer,3);
	adc_P = (I2C_rbuffer[0] << 12) | (I2C_rbuffer[1] << 4) | (I2C_rbuffer[2] >> 4);
	
	I2C_readByte(hi2cX,BMP280_ADDRESS,BMP280_TEMP_MSB,I2C_rbuffer,3);
	adc_T = (I2C_rbuffer[0] << 12) | (I2C_rbuffer[1] << 4) | (I2C_rbuffer[2] >> 4);
	
	// Temperature compensation
	int32_t t_fine;
	double var1, var2, p;
	var1 = (((double)adc_T)/16384.0 - ((double)dig.dig_T1)/1024.0) * ((double)dig.dig_T2);
  	var2 = ((((double)adc_T)/131072.0 - ((double)dig.dig_T1)/8192.0) * (((double)adc_T)/131072.0 - ((double)dig.dig_T1)/8192.0)) * ((double)dig.dig_T3);
  	t_fine = (int32_t)(var1 + var2);
  	*temperature = (double) (t_fine*10 / 5120.0)/10;
	
	
	// Pressure compensation
	var1 = ((double)t_fine/2.0) - 64000.0;
  	var2 = var1 * var1 * ((double)dig.dig_P6) / 32768.0;
  	var2 = var2 + var1 * ((double)dig.dig_P5) * 2.0;
  	var2 = (var2/4.0)+(((double)dig.dig_P4) * 65536.0);
  	var1 = (((double)dig.dig_P3) * var1 * var1 / 524288.0 + ((double)dig.dig_P2) * var1) / 524288.0;
  	var1 = (1.0 + var1 / 32768.0)*((double)dig.dig_P1);
  	if (var1 == 0.0)
  	{
    	return; // avoid exception caused by division by zero
  	}
  	p = 1048576.0 - (double)adc_P;
 	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  	var1 = ((double)dig.dig_P9) * p * p / 2147483648.0;
  	var2 = p * ((double)dig.dig_P8) / 32768.0;
  	p = (p + (var1 + var2 + ((double)dig.dig_P7)) / 16.0);

  	*pressure = (double)p;
  	
}

float BMP280_readAltitude (double pressure,float sealevelhPa)
{
	float altitude;
	pressure/=100;
	altitude= 44330*(1-pow(pressure/sealevelhPa,0.1903));
	return altitude;
} 

