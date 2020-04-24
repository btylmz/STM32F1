#include "MPU6050.h"
#include "MadgwickAHRS.h"
#include "math.h"


void I2C_writeByte(I2C_HandleTypeDef hi2cX,uint8_t device_address,uint8_t register_address,uint8_t * wdata,uint8_t size)
{
	HAL_I2C_Mem_Write(&hi2cX,device_address << 1,register_address,1,wdata,size,10*size);
}
void I2C_readByte(I2C_HandleTypeDef hi2cX,uint8_t device_address,uint8_t register_address,uint8_t * rdata,uint8_t size)
{
	HAL_I2C_Mem_Read(&hi2cX,device_address << 1,register_address,1,rdata,size,size*10);
}

float _aRes;
float _gRes;
float aRes,gRes;

uint8_t _Mmode;

float _accelBias[3],_gyroBias[3];

float _aX,_aY,_aZ,_gX,_gY,_gZ,_T;

uint8_t MPU6050_getID()
{
	uint8_t c ; 
	I2C_readByte(hi2cX,MPU6050_ADDRESS,WHO_AM_I_MPU6050,&c,1);
	return c;
}

void MPU6050_reset()
{
	uint8_t w = 0x80;
	I2C_writeByte(hi2cX,MPU6050_ADDRESS,PWR_MGMT_1,&w,1);
	HAL_Delay(100);
}

float MPU6050_getAres(uint8_t Ascale)
{
	switch(Ascale)
	{
	// Possible accelerometer scales (and their register bit settings) are:
  	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
         _aRes = 2.0f/32768.0f;
         return _aRes;
         break;
    case AFS_4G:
         _aRes = 4.0f/32768.0f;
         return _aRes;
         break;
    case AFS_8G:
         _aRes = 8.0f/32768.0f;
         return _aRes;
         break;
    case AFS_16G:
         _aRes = 16.0f/32768.0f;
         return _aRes;
         break;
		default : return 0;
	}
}

float MPU6050_getGres(uint8_t Gscale)
{
	switch(Gscale)
	{
	// Possible gyro scales (and their register bit settings) are:
  	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
    case GFS_250DPS:
          _gRes = 250.0/32768.0;
          return _gRes;
          break;
    case GFS_500DPS:
          _gRes = 500.0/32768.0;
          return _gRes;
          break;
    case GFS_1000DPS:
         _gRes = 1000.0/32768.0;
         return _gRes;
         break;
    case GFS_2000DPS:
          _gRes = 2000.0/32768.0;
         return _gRes;
         break;
		default : return 0;
	}
}


void MPU6050_Init(uint8_t Ascale, uint8_t Gscale, uint8_t sampleRate)
{
	uint8_t I2C_wbuffer[2];
	
	MPU6050_reset();
	
	// Wake up device
	I2C_wbuffer[0] = 0x00;												// Clear sleep mode bit (6), enable all sensors 
	I2C_writeByte(hi2cX,MPU6050_ADDRESS,PWR_MGMT_1,I2C_wbuffer,1);
	HAL_Delay(100);
	
	// get stable time source
	I2C_wbuffer[0] = 0x01; 												// Auto select clock source to be PLL gyroscope reference if ready else				
	I2C_writeByte(hi2cX,MPU6050_ADDRESS,PWR_MGMT_1,I2C_wbuffer,1); 
	HAL_Delay(100);	

 	// Configure Gyro and Thermometer
 	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 	// be higher than 1 / 0.0059 = 170 Hz
 	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 	// With the MPU6050, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz	
	I2C_wbuffer[0] = 0x06; 													
	I2C_writeByte(hi2cX,MPU6050_ADDRESS,CONFIG,I2C_wbuffer,1); 
	HAL_Delay(100);		
	
	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	I2C_wbuffer[0] = sampleRate; 													
	I2C_writeByte(hi2cX,MPU6050_ADDRESS,SMPLRT_DIV,I2C_wbuffer,1);  
																
  //	Set gyroscope full scale range                              
	I2C_wbuffer[0] = (0x00);// | (Gscale << 3);
	I2C_writeByte(hi2cX,MPU6050_ADDRESS,GYRO_CONFIG,I2C_wbuffer,1);
	 							                       		                                                  			
	// Set Accelerometer full-scale range configuration                              
	I2C_wbuffer[0] = (0x00);// | (Ascale << 3);
	I2C_writeByte(hi2cX,MPU6050_ADDRESS,ACCEL_CONFIG,I2C_wbuffer,1);
	
	// Set accelerometer sample rate configuration
 	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz 
	/*I2C_wbuffer[0] = 0x00;
	I2C_writeByte(hi2cX,MPU6050_ADDRESS,ACCEL_CONFIG2,I2C_wbuffer,1);	*/
	
	// Configure Interrupts and Bypass Enable
 	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
 	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
 	// can join the I2C bus and all can be controlled by the Arduino as master
	I2C_wbuffer[0] = 0x22;
	I2C_writeByte(hi2cX,MPU6050_ADDRESS,INT_PIN_CFG,I2C_wbuffer,1);
	HAL_Delay(100);
	
	I2C_wbuffer[0] = 0x01;
	I2C_writeByte(hi2cX,MPU6050_ADDRESS,INT_ENABLE,I2C_wbuffer,1);
	HAL_Delay(100);
}

void MPU6050_readAccelData(int16_t * destination)
{
	uint8_t rawData[6];	// x/y/z accel register data stored here
	I2C_readByte(hi2cX,MPU6050_ADDRESS,ACCEL_XOUT_H,rawData,6);
	
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void MPU6050_readGyroData(int16_t * destination)
{
	uint8_t rawData[6];	// x/y/z gyro register data stored here
	I2C_readByte(hi2cX,MPU6050_ADDRESS,GYRO_XOUT_H,rawData,6);
	
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void MPU6050_readTempData(int16_t * destination)
{
	uint8_t rawData[2];
	I2C_readByte(hi2cX,MPU6050_ADDRESS,TEMP_OUT_H,rawData,2);
	
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
}


void MPU6050_GyroAccel_Calibrate(float * dest1, float * dest2)
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  	uint16_t ii, packet_count, fifo_count;
  	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  	uint8_t I2C_wbuffer[3];
  	
  	MPU6050_reset();
  	
  	// Wake up device
		I2C_wbuffer[0] = 0x00;												// Clear sleep mode bit (6), enable all sensors 
		I2C_writeByte(hi2cX,MPU6050_ADDRESS,PWR_MGMT_1,I2C_wbuffer,1);
		HAL_Delay(100);
		
		// Enable the bypass bit
		I2C_wbuffer[0] = 0x22;												
		I2C_writeByte(hi2cX,MPU6050_ADDRESS,INT_PIN_CFG,I2C_wbuffer,1);
	
	
		I2C_wbuffer[0] = 0x00;												
		I2C_writeByte(hi2cX,MPU6050_ADDRESS,PWR_MGMT_2,I2C_wbuffer,1);
		HAL_Delay(100);
	
		I2C_wbuffer[0] = 0x00;
		I2C_writeByte(hi2cX,MPU6050_ADDRESS,INT_ENABLE,I2C_wbuffer,1);		// Disable all interrupts
		I2C_wbuffer[0] = 0x00;
		I2C_writeByte(hi2cX,MPU6050_ADDRESS,FIFO_EN,I2C_wbuffer,1);			// Disable FIFO
		I2C_wbuffer[0] = 0x00;
		I2C_writeByte(hi2cX,MPU6050_ADDRESS,PWR_MGMT_1,I2C_wbuffer,1);		// Turn on internal clock source
		I2C_wbuffer[0] = 0x00;
		I2C_writeByte(hi2cX,MPU6050_ADDRESS,I2C_MST_CTRL,I2C_wbuffer,1);	// Disable I2C master
		I2C_wbuffer[0] = 0x00;
		I2C_writeByte(hi2cX,MPU6050_ADDRESS,USER_CTRL,I2C_wbuffer,1);		// Disable FIFO and I2C master modes
		I2C_wbuffer[0] = 0x0C;
		I2C_writeByte(hi2cX,MPU6050_ADDRESS,USER_CTRL,I2C_wbuffer,1);		// Reset FIFO and DMP
		HAL_Delay(15);	
	
		// Configure MPU6050 gyro and accelerometer for bias calculation
		I2C_wbuffer[0] = 0x01;
		I2C_writeByte(hi2cX,MPU6050_ADDRESS,CONFIG,I2C_wbuffer,1);		// Set low-pass filter to 188 Hz
		I2C_wbuffer[0] = 0x00;
		I2C_writeByte(hi2cX,MPU6050_ADDRESS,SMPLRT_DIV,I2C_wbuffer,1);		// Set sample rate to 1 kHz
		I2C_wbuffer[0] = 0x00;
		I2C_writeByte(hi2cX,MPU6050_ADDRESS,GYRO_CONFIG,I2C_wbuffer,1);		// Set gyro full-scale to 250 degrees per second, maximum sensitivity
		I2C_wbuffer[0] = 0x00;
		I2C_writeByte(hi2cX,MPU6050_ADDRESS,ACCEL_CONFIG,I2C_wbuffer,1);		// Set accelerometer full-scale to 2 g, maximum sensitivity
	
	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g
  	
  	// Configure FIFO to capture accelerometer and gyro data for bias calculation
  	I2C_wbuffer[0] = 0x40;
		I2C_writeByte(hi2cX,MPU6050_ADDRESS,USER_CTRL,I2C_wbuffer,1);		// Enable FIFO  
		I2C_wbuffer[0] = 0x78;
		I2C_writeByte(hi2cX,MPU6050_ADDRESS,FIFO_EN,I2C_wbuffer,1);		// Enable gyro and accelerometer sensors for FIFO
	
		HAL_Delay(10);		// accumulate 40 samples in 40 milliseconds = 480 bytes ???
	
		// At end of sample accumulation, turn off FIFO sensor read
		I2C_wbuffer[0] = 0x00;
		I2C_writeByte(hi2cX,MPU6050_ADDRESS,FIFO_EN,I2C_wbuffer,1);		// Disable gyro and accelerometer sensors for FIFO
	
		I2C_readByte(hi2c1,MPU6050_ADDRESS,FIFO_COUNTH,data,2);					// read FIFO sample count
		fifo_count = ((uint16_t)data[0] << 8) | data[1];
		packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
	
		for (ii = 0; ii < packet_count; ii++) 
			{
			int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
			I2C_readByte(hi2cX,MPU6050_ADDRESS, FIFO_R_W,data, 12); // read data for averaging
			accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
			accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
			accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
			gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
			gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
			gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
			accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
			accel_bias[1] += (int32_t) accel_temp[1];
			accel_bias[2] += (int32_t) accel_temp[2];
			gyro_bias[0]  += (int32_t) gyro_temp[0];
			gyro_bias[1]  += (int32_t) gyro_temp[1];
			gyro_bias[2]  += (int32_t) gyro_temp[2];
            
	}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  	else {accel_bias[2] += (int32_t) accelsensitivity;}
  	
  	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  	data[3] = (-gyro_bias[1]/4)       & 0xFF;
  	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  	data[5] = (-gyro_bias[2]/4)       & 0xFF;
  	
  	// Push gyro biases to hardware registers
		I2C_writeByte(hi2c1,MPU6050_ADDRESS,XG_OFFSET_H,data,6);
	
	// Output scaled gyro biases for display in the main program
  	dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
  	dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  	dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;
		
		_gyroBias[0] = dest1[0];
		_gyroBias[1] = dest1[1];
		_gyroBias[2] = dest1[2];
	
	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.
	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	I2C_readByte(hi2c1,MPU6050_ADDRESS,XA_OFFSET_H,data,2);
	accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	I2C_readByte(hi2c1,MPU6050_ADDRESS,YA_OFFSET_H,data,2);
	accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	I2C_readByte(hi2c1,MPU6050_ADDRESS,ZA_OFFSET_H,data,2);
	accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	
	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  	for(ii = 0; ii < 3; ii++) 
	{
   	  if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  	}
	
	// Construct total accelerometer bias, including calculated average accelerometer bias from above
  	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  	accel_bias_reg[1] -= (accel_bias[1]/8);
  	accel_bias_reg[2] -= (accel_bias[2]/8);
  
  	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  	data[1] = (accel_bias_reg[0])      & 0xFF;
  	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  	data[3] = (accel_bias_reg[1])      & 0xFF;
  	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  	data[5] = (accel_bias_reg[2])      & 0xFF;
  	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	
	/*// Push accelerometer biases to hardware registers
	I2C_writeByte(MPU6050_ADDRESS,XA_OFFSET_H,&data[0],1);
	I2C_writeByte(MPU6050_ADDRESS,XA_OFFSET_L,&data[1],1);
	I2C_writeByte(MPU6050_ADDRESS,YA_OFFSET_H,&data[2],1);
	I2C_writeByte(MPU6050_ADDRESS,YA_OFFSET_L,&data[3],1);
	I2C_writeByte(MPU6050_ADDRESS,ZA_OFFSET_H,&data[4],1);
	I2C_writeByte(MPU6050_ADDRESS,ZA_OFFSET_L,&data[5],1);*/
	
	// Output scaled accelerometer biases for display in the main program
   	dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   	dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   	dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
		
		_accelBias[0] = dest2[0];
		_accelBias[1] = dest2[1];
		_accelBias[2] = dest2[2];
}


void MPU6050_readData(float * aX,float * aY,float * aZ,float * gX,float * gY,float * gZ,float * temperature)
{
	int16_t gyroRaw[3],accelRaw[3],magRaw[3],T;
	
	aRes = MPU6050_getAres(AFS_2G);
	gRes = MPU6050_getGres(GFS_250DPS);
	
	MPU6050_readGyroData(gyroRaw);
	*gX = (float)gyroRaw[0]*gRes-_gyroBias[0]; *gY = (float)gyroRaw[1]*gRes-_gyroBias[1]; *gZ = (float)gyroRaw[2]*gRes-_gyroBias[2];
	
	MPU6050_readAccelData(accelRaw);
	*aX = (float)accelRaw[0]*aRes-_accelBias[0]; *aY = (float)accelRaw[1]*aRes-_accelBias[1]; *aZ = (float)accelRaw[2]*aRes-_accelBias[2];
	
	MPU6050_readTempData(&T);
	*temperature = (float)T/333.87f + 21.0f;
	

}

void MPU6050_getEulerAngles(float * roll, float * pitch, float * yaw)
{
	float aX,aY,aZ,gX,gY,gZ,mX,mY,mZ,T;
	MPU6050_readData(&aX,&aY,&aZ,&gX,&gY,&gZ,&T);
	
	for(int i = 0; i<4; i++)
	{
		MadgwickAHRSupdateIMU(gX/rad_to_deg,gY/rad_to_deg,gZ/rad_to_deg,aX,aY,aZ);
	}
	
	*roll  = atan2f(2*(q0*q1 + q2*q3),1-2*(q1*q1 + q2*q2));
	*pitch = asinf(2*(q0*q2-q1*q3));
	*yaw	 = atan2f(2*(q0*q3 + q1*q2),1-2*(q2*q2 + q3*q3));

	*roll  *= rad_to_deg;
	*pitch *= rad_to_deg;
	*yaw   *= rad_to_deg;
}


