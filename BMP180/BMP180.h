#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "i2c.h"

#define BMP180_hi2cX hi2c1

//Slave Address of BMP180

#define BMP180_Address 0x77

// Registers

#define BMP180_out_msb   0xF6
#define BMP180_out_lsb   0xF7
#define BMP180_out_xsb 	 0xF8
#define BMP180_ctrl_meas 0xF4
#define BMP180_reset 	 0xE0
#define BMP180_id        0xD0

// Calibration Parameter Registers

#define BMP180_AC1_MSB   0xAA
#define BMP180_AC1_LSB   0xAB
#define BMP180_AC2_MSB   0xAC
#define BMP180_AC2_LSB   0xAD
#define BMP180_AC3_MSB   0xAE
#define BMP180_AC3_LSB   0xAF
#define BMP180_AC4_MSB   0xB0
#define BMP180_AC4_LSB   0xB1
#define BMP180_AC5_MSB   0xB2
#define BMP180_AC5_LSB   0xB3
#define BMP180_AC6_MSB   0xB4
#define BMP180_AC6_LSB   0xB5
#define BMP180_B1_MSB    0xB6
#define BMP180_B1_LSB    0xB7
#define BMP180_B2_MSB    0xB8
#define BMP180_B2_LSB    0xB9
#define BMP180_MB_MSB    0xBA
#define BMP180_MB_LSB    0xBB
#define BMP180_MC_MSB    0xBC
#define BMP180_MC_LSB    0xBD
#define BMP180_MD_MSB    0xBE
#define BMP180_MD_LSB    0xBF



// Measurement controls

#define BMP180_measure_Temperature 0x2E
#define BMP180_measure_Pressure_0  0x34
#define BMP180_measure_Pressure_1  0x74
#define BMP180_measure_Pressure_2  0xB4
#define BMP180_measure_Pressure_3  0xF4

#define seaLevelhPa 1004.5

typedef struct{
	
	short AC1;
	short AC2;
	short AC3;
	unsigned short AC4;
	unsigned short AC5;
	unsigned short AC6;
	
	short B1;
	short B2;
	
	short MB;
	short MC;
	short MD;
	
} BMP180_calib_data;

enum BMP180_sensor_sampling{
	
	BMP180_sampling_1 = 0x00,
	BMP180_sampling_2 = 0x01,
	BMP180_sampling_4 = 0x10,
	BMP180_sampling_8 = 0x11,
};

void BMP180_Init(enum BMP180_sensor_sampling sampling);
void BMP180_Reset(void);
long BMP180_getUncompensatedTemperature(void);
long BMP180_getUncompensatedPressure(void);
void BMP180_readTemperature(double * temperature);
void BMP180_readPressure(double * pressure);
void BMP180_readData(double * temperature,double * pressure);
double BMP180_readAltitude(double pressure,float seaLevel_hPa);


