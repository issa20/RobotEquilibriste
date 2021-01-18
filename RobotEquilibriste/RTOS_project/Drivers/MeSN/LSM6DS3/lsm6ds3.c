/******************************************************************************
SparkFunLSM6DS3.cpp
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

//See SparkFunLSM6DS3.h for additional topology notes.

#include "lsm6ds3.h"

#include "lsm6ds3_port.h"
/* Private variables ---------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
LSM6DS3_SensorSettings_t LSM6DS3_settings = {
		// Start with these default settings
		.gyroEnabled = 1,  //Can be 0 or 1
		.gyroRange = 2000,   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
		.gyroSampleRate = 416,   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
		.gyroBandWidth = 400,  //Hz.  Can be: 50, 100, 200, 400;
		.gyroFifoEnabled = 1,  //Set to include gyro in FIFO
		.gyroFifoDecimation = 1,  //set 1 for on /1

		.accelEnabled = 1,
		.accelODROff = 1,
		.accelRange = 16,      //Max G force readable.  Can be: 2, 4, 8, 16
		.accelSampleRate = 416,  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
		.accelBandWidth = 100,  //Hz.  Can be: 50, 100, 200, 400;
		.accelFifoEnabled = 1,  //Set to include accelerometer in the FIFO
		.accelFifoDecimation = 1,  //set 1 for on /1

		.tempEnabled = 1,

		//Select interface mode
		.commMode = 1,  //Can be modes 1, 2 or 3

		//FIFO control data
		.fifoThreshold = 3000,  //Can be 0 to 4096 (16 bit bytes)
		.fifoSampleRate = 10,  //default 10Hz
		.fifoModeWord = 0,  //Default off

		//Error checking
		.allOnesCounter = 0,
		.nonSuccessCounter = 0
};

/* Private function prototypes -----------------------------------------------*/
LSM6DS3_status_t LSM6DS3_begin_accel();
LSM6DS3_status_t LSM6DS3_begin_gyro();

/* Public functions ----------------------------------------------------------*/

LSM6DS3_status_t LSM6DS3Core_embeddedPage( void )
{
	LSM6DS3_status_t returnError = LSM6DS3Core_writeRegister( LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x80 );

	return returnError;
}

LSM6DS3_status_t LSM6DS3Core_basePage( void )
{
	LSM6DS3_status_t returnError = LSM6DS3Core_writeRegister( LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x00 );

	return returnError;
}


//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as "myIMU.settings.commInterface = SPI_MODE;" or
//  "myIMU.settings.accelEnabled = 1;" to configure before calling .begin();
//
//****************************************************************************//
LSM6DS3_status_t LSM6DS3_begin(uint8_t* result)
{
	LSM6DS3_status_t returnError = IMU_SUCCESS;

	//Begin the inherited core.  This gets the physical wires connected
	returnError = LSM6DS3Core_beginCore();

	//Check the settings structure values to determine how to setup the device
	//Setup the accelerometer******************************
	if ( (returnError == IMU_SUCCESS) && (LSM6DS3_settings.accelEnabled == 1) ) {
		returnError = LSM6DS3_begin_accel();
	}
	//Setup the gyroscope**********************************************
	if ( (returnError == IMU_SUCCESS) && (LSM6DS3_settings.gyroEnabled == 1) ) {
		returnError = LSM6DS3_begin_gyro();
	}
	//Setup the internal temperature sensor
	if ( (returnError == IMU_SUCCESS) && (LSM6DS3_settings.tempEnabled == 1) ) {
		//Nothing to do : always on
	}

	//Return WHO AM I reg  //Not no mo!
	if (returnError == IMU_SUCCESS){
		returnError = LSM6DS3Core_readRegister(result, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);
	}

	return returnError;
}

//****************************************************************************//
//
//  Accelerometer section
//
//****************************************************************************//
LSM6DS3_status_t LSM6DS3_readRawAccelX( int16_t* pData )
{
	return LSM6DS3Core_readRegisterInt16( pData, LSM6DS3_ACC_GYRO_OUTX_L_XL );
}
LSM6DS3_status_t LSM6DS3_readMgAccelX( int32_t* pData )
{
	int16_t value;
	LSM6DS3_status_t status = IMU_SUCCESS;

	status = LSM6DS3_readRawAccelX(&value);

	if(status == IMU_SUCCESS){
		*pData = LSM6DS3_calcAccel(value);
	}

	return status;
}

LSM6DS3_status_t LSM6DS3_readRawAccelY( int16_t* pData )
{
	return LSM6DS3Core_readRegisterInt16( pData, LSM6DS3_ACC_GYRO_OUTY_L_XL );
}
LSM6DS3_status_t LSM6DS3_readMgAccelY( int32_t* pData )
{
	int16_t value;
	LSM6DS3_status_t status = IMU_SUCCESS;

	status = LSM6DS3_readRawAccelY(&value);

	if(status == IMU_SUCCESS){
		*pData = LSM6DS3_calcAccel(value);
	}

	return status;
}

LSM6DS3_status_t LSM6DS3_readRawAccelZ( int16_t* pData )
{
	return LSM6DS3Core_readRegisterInt16( pData, LSM6DS3_ACC_GYRO_OUTZ_L_XL );
}
LSM6DS3_status_t LSM6DS3_readMgAccelZ( int32_t* pData )
{
	int16_t value;
	LSM6DS3_status_t status = IMU_SUCCESS;

	status = LSM6DS3_readRawAccelZ(&value);

	if(status == IMU_SUCCESS){
		*pData = LSM6DS3_calcAccel(value);
	}

	return status;
}

//Compute acceleration value in milli-g from raw data
int32_t LSM6DS3_calcAccel( int16_t input )
{
	return (((int32_t)input * 61 * (LSM6DS3_settings.accelRange >> 1)) / 1000);
}

//****************************************************************************//
//
//  Gyroscope section
//
//****************************************************************************//
LSM6DS3_status_t LSM6DS3_readRawGyroX( int16_t* pData )
{
	return LSM6DS3Core_readRegisterInt16( pData, LSM6DS3_ACC_GYRO_OUTX_L_G );
}
LSM6DS3_status_t LSM6DS3_readMdpsGyroX( int32_t* pData )
{
	int16_t value;
	LSM6DS3_status_t status = IMU_SUCCESS;

	status = LSM6DS3_readRawGyroX(&value);

	if(status == IMU_SUCCESS){
		*pData = LSM6DS3_calcGyro(value);
	}

	return status;
}

LSM6DS3_status_t LSM6DS3_readRawGyroY( int16_t* pData )
{
	return LSM6DS3Core_readRegisterInt16( pData, LSM6DS3_ACC_GYRO_OUTY_L_G );
}
LSM6DS3_status_t LSM6DS3_readMdpsGyroY( int32_t* pData )
{
	int16_t value;
	LSM6DS3_status_t status = IMU_SUCCESS;

	status = LSM6DS3_readRawGyroY(&value);

	if(status == IMU_SUCCESS){
		*pData = LSM6DS3_calcGyro(value);
	}

	return status;
}

LSM6DS3_status_t LSM6DS3_readRawGyroZ( int16_t* pData )
{
	return LSM6DS3Core_readRegisterInt16( pData, LSM6DS3_ACC_GYRO_OUTZ_L_G );
}
LSM6DS3_status_t LSM6DS3_readMdpsGyroZ( int32_t* pData )
{
	int16_t value;
	LSM6DS3_status_t status = IMU_SUCCESS;

	status = LSM6DS3_readRawGyroZ(&value);

	if(status == IMU_SUCCESS){
		*pData = LSM6DS3_calcGyro(value);
	}

	return status;
}

//Compute rotation value in milli-deg/sec from raw data
int32_t LSM6DS3_calcGyro( int16_t input )
{
	uint8_t gyroRangeDivisor = LSM6DS3_settings.gyroRange / 125;
	if ( LSM6DS3_settings.gyroRange == 245 ) {
		gyroRangeDivisor = 2;
	}

	return ( (int32_t)input * 4375 * (gyroRangeDivisor) / 1000 );
}

//****************************************************************************//
//
//  Temperature section
//
//****************************************************************************//
int16_t LSM6DS3_readRawTemp( void )
{
	int16_t output;
	LSM6DS3Core_readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUT_TEMP_L );
	return output;
}

float LSM6DS3_readTempC( void )
{
	float output = (float)LSM6DS3_readRawTemp() / 16; //divide by 16 to scale
	output += 25; //Add 25 degrees to remove offset

	return output;

}

float LSM6DS3_readTempF( void )
{
	float output = (float)LSM6DS3_readRawTemp() / 16; //divide by 16 to scale
	output += 25; //Add 25 degrees to remove offset
	output = (output * 9) / 5 + 32;

	return output;

}

//****************************************************************************//
//
//  FIFO section
//
//****************************************************************************//
void LSM6DS3_fifoBegin( void ) {
	//CONFIGURE THE VARIOUS FIFO SETTINGS
	//
	//
	//This section first builds a bunch of config words, then goes through
	//and writes them all.

	//Split and mask the threshold
	uint8_t thresholdLByte = LSM6DS3_settings.fifoThreshold & 0x00FF;
	uint8_t thresholdHByte = (LSM6DS3_settings.fifoThreshold & 0x0F00) >> 8;
	//Pedo bits not configured (ctl2)

	//CONFIGURE FIFO_CTRL3
	uint8_t tempFIFO_CTRL3 = 0;
	if (LSM6DS3_settings.gyroFifoEnabled == 1)
	{
		//Set up gyro stuff
		//Build on FIFO_CTRL3
		//Set decimation
		tempFIFO_CTRL3 |= (LSM6DS3_settings.gyroFifoDecimation & 0x07) << 3;

	}
	if (LSM6DS3_settings.accelFifoEnabled == 1)
	{
		//Set up accelerometer stuff
		//Build on FIFO_CTRL3
		//Set decimation
		tempFIFO_CTRL3 |= (LSM6DS3_settings.accelFifoDecimation & 0x07);
	}

	//CONFIGURE FIFO_CTRL4  (nothing for now-- sets data sets 3 and 4
	uint8_t tempFIFO_CTRL4 = 0;


	//CONFIGURE FIFO_CTRL5
	uint8_t tempFIFO_CTRL5 = 0;
	switch (LSM6DS3_settings.fifoSampleRate) {
	default:  //set default case to 10Hz(slowest)
	case 10:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_10Hz;
		break;
	case 25:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_25Hz;
		break;
	case 50:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_50Hz;
		break;
	case 100:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_100Hz;
		break;
	case 200:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_200Hz;
		break;
	case 400:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_400Hz;
		break;
	case 800:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_800Hz;
		break;
	case 1600:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_1600Hz;
		break;
	case 3300:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_3300Hz;
		break;
	case 6600:
		tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_6600Hz;
		break;
	}
	//Hard code the fifo mode here:
	tempFIFO_CTRL5 |= LSM6DS3_settings.fifoModeWord = 6;  //set mode:

	//Write the data
	LSM6DS3Core_writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL1, thresholdLByte);
	//Serial.println(thresholdLByte, HEX);
	LSM6DS3Core_writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL2, thresholdHByte);
	//Serial.println(thresholdHByte, HEX);
	LSM6DS3Core_writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL3, tempFIFO_CTRL3);
	LSM6DS3Core_writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL4, tempFIFO_CTRL4);
	LSM6DS3Core_writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, tempFIFO_CTRL5);

}
void LSM6DS3_fifoClear( void ) {
	//Drain the fifo data and dump it
	while( (LSM6DS3_fifoGetStatus() & 0x1000 ) == 0 ) {
		LSM6DS3_fifoRead();
	}

}
int16_t LSM6DS3_fifoRead( void ) {
	//Pull the last data from the fifo
	uint8_t tempReadByte = 0;
	uint16_t tempAccumulator = 0;
	LSM6DS3Core_readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L);
	tempAccumulator = tempReadByte;
	LSM6DS3Core_readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_H);
	tempAccumulator |= ((uint16_t)tempReadByte << 8);

	return tempAccumulator;
}

uint16_t LSM6DS3_fifoGetStatus( void ) {
	//Return some data on the state of the fifo
	uint8_t tempReadByte = 0;
	uint16_t tempAccumulator = 0;
	LSM6DS3Core_readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_STATUS1);
	tempAccumulator = tempReadByte;
	LSM6DS3Core_readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_STATUS2);
	tempAccumulator |= (tempReadByte << 8);

	return tempAccumulator;

}
void LSM6DS3_fifoEnd( void ) {
	// turn off the fifo
	LSM6DS3Core_writeRegister(LSM6DS3_ACC_GYRO_FIFO_STATUS1, 0x00);  //Disable
}

/* Private functions -------------------------------------------------------------*/

LSM6DS3_status_t LSM6DS3_begin_accel(){

	uint8_t dataToWrite = 0;	//Temporary variable
	LSM6DS3_status_t status = IMU_SUCCESS;

	//Build config reg
	//First patch in filter bandwidth
	switch (LSM6DS3_settings.accelBandWidth) {
	case 50:
		dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_50Hz;
		break;
	case 100:
		dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
		break;
	case 200:
		dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
		break;
	default:  //set default case to max passthrough
	case 400:
		dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_400Hz;
		break;
	}

	//Next, patch in full scale
	switch (LSM6DS3_settings.accelRange) {
	case 2:
		dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
		break;
	case 4:
		dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_4g;
		break;
	case 8:
		dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_8g;
		break;
	default:  //set default case to 16(max)
	case 16:
		dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_16g;
		break;
	}

	//Lastly, patch in accelerometer ODR
	switch (LSM6DS3_settings.accelSampleRate) {
	case 13:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_13Hz;
		break;
	case 26:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;
		break;
	case 52:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_52Hz;
		break;
	default:  //Set default to 104
	case 104:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
		break;
	case 208:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_208Hz;
		break;
	case 416:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
		break;
	case 833:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_833Hz;
		break;
	case 1660:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_1660Hz;
		break;
	case 3330:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_3330Hz;
		break;
	case 6660:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_6660Hz;
		break;
	case 13330:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_13330Hz;
		break;
	}

	//Now, write the patched together data
	status = LSM6DS3Core_writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

	//Set the ODR bit
	if(status == IMU_SUCCESS){
		status = LSM6DS3Core_readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
	}
	if(status == IMU_SUCCESS){
		dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);
		if ( LSM6DS3_settings.accelODROff == 1) {
			dataToWrite |= LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED;
		}
		status = LSM6DS3Core_writeRegister(LSM6DS3_ACC_GYRO_CTRL4_C, dataToWrite);
	}

	return status;
}

LSM6DS3_status_t LSM6DS3_begin_gyro(){

	uint8_t dataToWrite = 0; //Temporary variable
	LSM6DS3_status_t status = IMU_SUCCESS;

	//Build config reg
	//First, patch in full scale
	switch (LSM6DS3_settings.gyroRange) {
	case 125:
		dataToWrite |= LSM6DS3_ACC_GYRO_FS_125_ENABLED;
		break;
	case 245:
		dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_245dps;
		break;
	case 500:
		dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_500dps;
		break;
	case 1000:
		dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_1000dps;
		break;
	default:  //Default to full 2000DPS range
	case 2000:
		dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_2000dps;
		break;
	}
	//Lastly, patch in gyro ODR
	switch (LSM6DS3_settings.gyroSampleRate) {
	case 13:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_13Hz;
		break;
	case 26:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_26Hz;
		break;
	case 52:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_52Hz;
		break;
	default:  //Set default to 104
	case 104:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_104Hz;
		break;
	case 208:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_208Hz;
		break;
	case 416:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_416Hz;
		break;
	case 833:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_833Hz;
		break;
	case 1660:
		dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_1660Hz;
		break;
	}

	//Write the byte
	status = LSM6DS3Core_writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite);

	return status;

}
