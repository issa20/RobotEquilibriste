/******************************************************************************
SparkFunLSM6DS3.h
LSM6DS3 Arduino and Teensy Driver

Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/LSM6DS3_Breakout
https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation
Either can be omitted if not used

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __LSM6DS3IMU_H__
#define __LSM6DS3IMU_H__

#include "stdint.h"

#include "lsm6ds3_status.h"

//Change to embedded page
LSM6DS3_status_t LSM6DS3Core_embeddedPage( void );
//Change to base page
LSM6DS3_status_t LSM6DS3Core_basePage( void );

//This structure holds the settings the driver uses to do calculations
typedef struct
{
	//Gyro settings
	uint8_t gyroEnabled;
	uint16_t gyroRange;
	uint16_t gyroSampleRate;
	uint16_t gyroBandWidth;

	uint8_t gyroFifoEnabled;
	uint8_t gyroFifoDecimation;

	//Accelerometer settings
	uint8_t accelEnabled;
	uint8_t accelODROff;
	uint16_t accelRange;
	uint16_t accelSampleRate;
	uint16_t accelBandWidth;

	uint8_t accelFifoEnabled;
	uint8_t accelFifoDecimation;

	//Temperature settings
	uint8_t tempEnabled;

	//Non-basic mode settings
	uint8_t commMode;

	//FIFO control data
	uint16_t fifoThreshold;
	int16_t fifoSampleRate;
	uint8_t fifoModeWord;

	//Error checking
	uint16_t allOnesCounter;
	uint16_t nonSuccessCounter;

}LSM6DS3_SensorSettings_t;

extern LSM6DS3_SensorSettings_t LSM6DS3_settings;

//Call to apply SensorSettings
LSM6DS3_status_t LSM6DS3_begin(uint8_t* result);

//Read the raw bits from the sensor cast as 16-bit signed integers
LSM6DS3_status_t LSM6DS3_readRawAccelX( int16_t* pData );
LSM6DS3_status_t LSM6DS3_readRawAccelY( int16_t* pData );
LSM6DS3_status_t LSM6DS3_readRawAccelZ( int16_t* pData );
LSM6DS3_status_t LSM6DS3_readRawGyroX( int16_t* pData );
LSM6DS3_status_t LSM6DS3_readRawGyroY( int16_t* pData );
LSM6DS3_status_t LSM6DS3_readRawGyroZ( int16_t* pData );

//Returns the values in milli-g as 32-bit integer.  Inside, this calls readRaw___();
LSM6DS3_status_t LSM6DS3_readMgAccelX( int32_t* pData );
LSM6DS3_status_t LSM6DS3_readMgAccelY( int32_t* pData );
LSM6DS3_status_t LSM6DS3_readMgAccelZ( int32_t* pData );
LSM6DS3_status_t LSM6DS3_readMdpsGyroX( int32_t* pData );
LSM6DS3_status_t LSM6DS3_readMdpsGyroY( int32_t* pData );
LSM6DS3_status_t LSM6DS3_readMdpsGyroZ( int32_t* pData );

//Temperature related functions
int16_t LSM6DS3_readRawTemp( void );
float LSM6DS3_readTempC( void );
float LSM6DS3_readTempF( void );

//FIFO stuff
void LSM6DS3_fifoBegin( void );
void LSM6DS3_fifoClear( void );
int16_t LSM6DS3_fifoRead( void );
uint16_t LSM6DS3_fifoGetStatus( void );
void LSM6DS3_fifoEnd( void );

int32_t LSM6DS3_calcGyro( int16_t );
int32_t LSM6DS3_calcAccel( int16_t );

#endif  // End of __LSM6DS3IMU_H__ definition check
