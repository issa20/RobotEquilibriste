
#ifndef __LSM6DS3_status_H__
#define __LSM6DS3_status_H__

// Return values
typedef enum
{
	IMU_SUCCESS,
	IMU_HW_ERROR,
	IMU_NOT_SUPPORTED,
	IMU_GENERIC_ERROR,
	IMU_OUT_OF_BOUNDS,
	IMU_ALL_ONES_WARNING,
	//...
} LSM6DS3_status_t;

#endif  // End of __LSM6DS3_status_H__ definition check
