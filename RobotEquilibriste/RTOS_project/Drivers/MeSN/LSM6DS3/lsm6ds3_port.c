
#include "lsm6ds3_port.h"

#include "MeSN_i2c.h"

LSM6DS3_status_t LSM6DS3Core_beginCore(void)
{
	LSM6DS3_status_t returnError = IMU_SUCCESS;
	volatile uint8_t readCheck = 0;

#if defined (I2C_MODE)
	// start the i2c library
	MeSN_I2C_Init();
#elif defined (SPI_MODE)
	// start the SPI library:
	// initalize the  data ready and chip select pins:
#else
	#error "Please select interface for LSM6DS3 IMU (I2C or SPI) in lsm6ds3_port.h"
#endif

	//Spin for a few ms
	HAL_Delay(300);

	//Check the ID register to determine if the operation was a success.
	if (LSM6DS3Core_readRegister((uint8_t*)&readCheck, LSM6DS3_ACC_GYRO_WHO_AM_I_REG) == IMU_SUCCESS){
		if( readCheck != 0x69 )
		{
			returnError = IMU_HW_ERROR;
		}
	}

	return returnError;

}

//****************************************************************************//
//
//  ReadRegister
//
//  Parameters:
//    *outputPointer -- Pass &variable (address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
LSM6DS3_status_t LSM6DS3Core_readRegister(uint8_t* outputPointer, uint8_t offset) {
	LSM6DS3_status_t returnError = IMU_SUCCESS;

#if defined (I2C_MODE)
	if(MeSN_I2C_RegisterRead(I2CAddress, offset, 1, outputPointer, 1) != USER_OK){
		returnError = IMU_HW_ERROR;
	}
//	Wire.beginTransmission(I2CAddress);
//	Wire.write(offset);
//	if( Wire.endTransmission() != 0 )
//	{
//		returnError = IMU_HW_ERROR;
//	}
//	Wire.requestFrom(I2CAddress, numBytes);
//	while ( Wire.available() ) // slave may send less than requested
//	{
//		result = Wire.read(); // receive a byte as a proper uint8_t
//	}
#elif defined (SPI_MODE)
		// take the chip select low to select the device:
		digitalWrite(chipSelectPin, LOW);
		// send the device the register you want to read:
		SPI.transfer(offset | 0x80);  //Ored with "read request" bit
		// send a value of 0 to read the first byte returned:
		result = SPI.transfer(0x00);
		// take the chip select high to de-select:
		digitalWrite(chipSelectPin, HIGH);

		if( result == 0xFF )
		{
			//we've recieved all ones, report
			returnError = IMU_ALL_ONES_WARNING;
		}
#endif

	return returnError;
}

//****************************************************************************//
//
//  ReadRegisterRegion
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//    length -- number of bytes to read
//
//  Note:  Does not know if the target memory space is an array or not, or
//    if there is the array is big enough.  if the variable passed is only
//    two bytes long and 3 bytes are requested, this will over-write some
//    other memory!
//
//****************************************************************************//
LSM6DS3_status_t LSM6DS3Core_readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	LSM6DS3_status_t returnError = IMU_SUCCESS;

#if defined (I2C_MODE)
	if(MeSN_I2C_RegisterRead(I2CAddress, offset, 1, outputPointer, length) != USER_OK){
		returnError = IMU_HW_ERROR;
	}
//	Wire.beginTransmission(I2CAddress);
//	Wire.write(offset);
//	if( Wire.endTransmission() != 0 )
//	{
//		returnError = IMU_HW_ERROR;
//	}
//	else  //OK, all worked, keep going
//	{
//		// request 6 bytes from slave device
//		Wire.requestFrom(I2CAddress, length);
//		while ( (Wire.available()) && (i < length))  // slave may send less than requested
//		{
//			c = Wire.read(); // receive a byte as character
//			*outputPointer = c;
//			outputPointer++;
//			i++;
//		}
//	}
#elif defined (SPI_MODE)
	// take the chip select low to select the device:
	digitalWrite(chipSelectPin, LOW);
	// send the device the register you want to read:
	SPI.transfer(offset | 0x80);  //Ored with "read request" bit
	while ( i < length ) // slave may send less than requested
	{
		c = SPI.transfer(0x00); // receive a byte as character
		if( c == 0xFF )
		{
			//May have problem
			tempFFCounter++;
		}
		*outputPointer = c;
		outputPointer++;
		i++;
	}
	if( tempFFCounter == i )
	{
		//Ok, we've recieved all ones, report
		returnError = IMU_ALL_ONES_WARNING;
	}
	// take the chip select high to de-select:
	digitalWrite(chipSelectPin, HIGH);
#endif

	return returnError;
}

//****************************************************************************//
//
//  readRegisterInt16
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
LSM6DS3_status_t LSM6DS3Core_readRegisterInt16( int16_t* outputPointer, uint8_t offset )
{
	uint8_t myBuffer[2];
	LSM6DS3_status_t returnError;

	returnError = LSM6DS3Core_readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
	*outputPointer = (((int16_t)myBuffer[1]) << 8) | ((int16_t)myBuffer[0]);

	return returnError;
}

//****************************************************************************//
//
//  writeRegister
//
//  Parameters:
//    offset -- register to write
//    dataToWrite -- 8 bit data to write to register
//
//****************************************************************************//
LSM6DS3_status_t LSM6DS3Core_writeRegister(uint8_t offset, uint8_t dataToWrite) {
	LSM6DS3_status_t returnError = IMU_SUCCESS;

	uint8_t bufferToSend[2];

	bufferToSend[0] = offset;
	bufferToSend[1] = dataToWrite;

#if defined (I2C_MODE)
	//Write the byte
	MeSN_I2C_Write(I2CAddress, bufferToSend, 2);
//	Wire.beginTransmission(I2CAddress);
//	Wire.write(offset);
//	Wire.write(dataToWrite);
//	if( Wire.endTransmission() != 0 )
//	{
//		returnError = IMU_HW_ERROR;
//	}
#elif defined (SPI_MODE)
	// take the chip select low to select the device:
	digitalWrite(chipSelectPin, LOW);
	// send the device the register you want to read:
	SPI.transfer(offset);
	// send a value of 0 to read the first byte returned:
	SPI.transfer(dataToWrite);
	// decrement the number of bytes left to read:
	// take the chip select high to de-select:
	digitalWrite(chipSelectPin, HIGH);
#endif

	return returnError;
}

