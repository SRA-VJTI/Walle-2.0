/**
	MPU library for WALL-E 2.0
	This module is used to get raw values of 
	acceleration components in the direction of the X-Axis, Y-Axis and Z-Axis,
	and the raw values on angular velocities about the X-Axis, Y-Axis and Z-Axis.

	For more information visit http://sra.vjti.info/

	This code is in the public domain.

	modified 29 May, 2017
	by Society Of Robotics And Automation, VJTI.
*/

/*Can also use #pragma once*/
#ifndef MPU_H
#define MPU_H

/*TODO: See if you need the cpp gaurds
#ifdef __cplusplus
extern "C" {
#endif
*/
#include <Wire.h>	//	For I2C Communication

/* 	The Structure that is used to store raw values read from the MPU	*/
struct raw_data {
	int16_t	raw_acce_x;
    int16_t raw_acce_y;
    int16_t raw_acce_z;
    int16_t temp;
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
};

/* 	The Structure that is used to store initial data read from the MPU	*/
struct initial_data {
	float acce_angle;
	long  gyro_reading;
};                                  

/* 	A function to begin I2C transmission with MPU-6050
	Wakes up the MPU by clearing the PWR_MGMT_1 register*/
void start_mpu(void);      

/* 	A function to read raw values from the registers of the MPU-6050	
	Reads raw values from the registers of the MPU	*/
void read_raw_values_mpu(struct raw_data*);                             

/* 	A function to get initial raw values from both the gyroscope and accelerometer
	Gets initial raw reading from gyroscope and initial angle calculated by accelerometer	*/
void calibrate_mpu(struct initial_data*);    

/*TODO: See if you need the cpp gaurds
#ifdef __cplusplus
}
#endif
*/

#endif
