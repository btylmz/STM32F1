#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "i2c.h"

// Select "Generate peripheral initialization as a pair of .c/.h filer per peripheral" in the cubemx
// This library uses I2C.
// Add I2C_writeByte and I2C_readByte functions to the i2c.c
#define hi2cX hi2c1  // define the hi2cX 


// Define the SDO value. SDO pin high -> 1, SDO pin low -> 0
#define SDO 0
#if SDO 
#define BMP280_ADDRESS 0x77   // Device address when SDO = 1
#else
#define BMP280_ADDRESS 0x76   // Device address when ADO = 0
#endif  

#define BMP280_ID 0xD0
#define BMP280_RESET 0xE0
#define BMP280_STATUS 0xF3
#define BMP280_CTRL_MEAS 0xF4
#define BMP280_CONFIG 0xF5
#define BMP280_PRESS_MSB 0xF7
#define BMP280_PRESS_LSB 0xF8
#define BMP280_PRESS_XSB 0xF9
#define BMP280_TEMP_MSB 0xFA
#define BMP280_TEMP_LSB 0xFB
#define BMP280_TEMP_XSB 0xFC
#define BMP280_digT1_MSB 0x89
#define BMP280_digT1_LSB 0x88
#define BMP280_digT2_MSB 0x8B
#define BMP280_digT2_LSB 0x8A
#define BMP280_digT3_MSB 0x8D
#define BMP280_digT3_LSB 0x8C
#define BMP280_digP1_MSB 0x8F
#define BMP280_digP1_LSB 0x8E
#define BMP280_digP2_MSB 0x91
#define BMP280_digP2_LSB 0x90
#define BMP280_digP3_MSB 0x93
#define BMP280_digP3_LSB 0x92
#define BMP280_digP4_MSB 0x95
#define BMP280_digP4_LSB 0x94
#define BMP280_digP5_MSB 0x97
#define BMP280_digP5_LSB 0x96
#define BMP280_digP6_MSB 0x99
#define BMP280_digP6_LSB 0x98
#define BMP280_digP7_MSB 0x9B
#define BMP280_digP7_LSB 0x9A
#define BMP280_digP8_MSB 0x9D
#define BMP280_digP8_LSB 0x9C
#define BMP280_digP9_MSB 0x9F
#define BMP280_digP9_LSB 0x9E
#define BMP280_RESERVED_MSB 0xA1
#define BMP280_RESERVED_LSB 0xA0

// Struct to hold calibration data.
  typedef struct {
	
  uint16_t dig_T1; /**< dig_T1 cal register. */
  int16_t dig_T2;  /**<  dig_T2 cal register. */
  int16_t dig_T3;  /**< dig_T3 cal register. */

  uint16_t dig_P1; /**< dig_P1 cal register. */
  int16_t dig_P2;  /**< dig_P2 cal register. */
  int16_t dig_P3;  /**< dig_P3 cal register. */
  int16_t dig_P4;  /**< dig_P4 cal register. */
  int16_t dig_P5;  /**< dig_P5 cal register. */
  int16_t dig_P6;  /**< dig_P6 cal register. */
  int16_t dig_P7;  /**< dig_P7 cal register. */
  int16_t dig_P8;  /**< dig_P8 cal register. */
  int16_t dig_P9;  /**< dig_P9 cal register. */

  uint8_t dig_H1; /**< dig_H1 cal register. */
  int16_t dig_H2; /**< dig_H2 cal register. */
  uint8_t dig_H3; /**< dig_H3 cal register. */
  int16_t dig_H4; /**< dig_H4 cal register. */
  int16_t dig_H5; /**< dig_H5 cal register. */
  int8_t dig_H6;  /**< dig_H6 cal register. */
} BMP280_calib_data;

enum BMP280_sensor_sampling {
    /** No over-sampling. */
   BMP280_SAMPLING_NONE = 0x00,
    /** 1x over-sampling. */
   BMP280_SAMPLING_X1 = 0x01,
    /** 2x over-sampling. */
   BMP280_SAMPLING_X2 = 0x02,
    /** 4x over-sampling. */
   BMP280_SAMPLING_X4 = 0x03,
    /** 8x over-sampling. */
   BMP280_SAMPLING_X8 = 0x04,
    /** 16x over-sampling. */
   BMP280_SAMPLING_X16 = 0x05
  };
  
  
  /** Operating mode for the sensor. */
  enum BMP280_sensor_mode {
    /** Sleep mode. */
    BMP280_MODE_SLEEP = 0x00,
    /** Forced mode. */
    BMP280_MODE_FORCED = 0x01,
    /** Normal mode. */
    BMP280_MODE_NORMAL = 0x03,
    /** Software reset. */
    BMP280_MODE_SOFT_RESET_CODE = 0xB6
  };

  /** Filtering level for sensor data. */
  enum BMP280_sensor_filter {
    /** No filtering. */
    BMP280_FILTER_OFF = 0x00,
    /** 2x filtering. */
    BMP280_FILTER_X2 = 0x01,
    /** 4x filtering. */
    BMP280_FILTER_X4 = 0x02,
    /** 8x filtering. */
    BMP280_FILTER_X8 = 0x03,
    /** 16x filtering. */
    BMP280_FILTER_X16 = 0x04
  };

  /** Standby duration in ms */
  enum BMP280_standby_duration {
    /** 1 ms standby. */
    BMP280_STANDBY_MS_1 = 0x00,
    /** 63 ms standby. */
    BMP280_STANDBY_MS_63 = 0x01,
    /** 125 ms standby. */
    BMP280_STANDBY_MS_125 = 0x02,
    /** 250 ms standby. */
    BMP280_STANDBY_MS_250 = 0x03,
    /** 500 ms standby. */
    BMP280_STANDBY_MS_500 = 0x04,
    /** 1000 ms standby. */
    BMP280_STANDBY_MS_1000 = 0x05,
    /** 2000 ms standby. */
    BMP280_STANDBY_MS_2000 = 0x06,
    /** 4000 ms standby. */
    BMP280_STANDBY_MS_4000 = 0x07
  };
  
  enum BMP280_status {	
  	BMP280_im_update = 0x01,
  	
  	BMP280_measuring = 0x02
  };
  
  uint8_t BMP280_getID(void);
  uint8_t BMP280_getStatus(void);
  void BMP280_Init(enum BMP280_sensor_mode mode,enum BMP280_sensor_sampling tempSampling,enum BMP280_sensor_sampling pressSampling,enum BMP280_sensor_filter filter,enum BMP280_standby_duration duration);
  void BMP280Temp_Pres_Calc(double* temperature, double* pressure);
  float BMP280_readAltitude (double pressure,float sealevelhPa);	
  

