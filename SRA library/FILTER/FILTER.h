/**
	FILTER library for WALL-E 2.0
	This module is used to convert the raw readings 
	from the MPU module into usable filtered readings.
	The required filter can be chosen.
	The available filters are:
		1)	None	
		2)	Complementary
		3)	Kalman

	For more information visit http://sra.vjti.info/

	This code is in the public domain.

	modified 29 May, 2017
	by Society Of Robotics And Automation, VJTI.
*/

/*Can also use #pragma once*/
#ifndef FILTER_H
#define FILTER_H

/*
#ifdef __cplusplus
extern "C" {
#endif
*/

#include <MPU.h>

enum filter_type {
    NONE          = 1,  						//	NONE filter directly reads from MPU-6050
    COMPLEMENTARY = 2,  						//	Applies Complementary filter to the raw readings
    KALMAN        = 3  							//	Applies Kalman filter to the raw readings 
};

static long	prev_time = 0;                		//	A variable to hold the time elapsed till previous iteration

/*	This is called when applying complementary filter to set Gyro Weight	*/
void  complementary_init(float); 

/*	Calculates angular Velocity using Gyro data	*/
float calc_angular_velocity(struct raw_data, struct initial_data); 

/*	Calculates angle only using Gyroscope	*/
float calc_gyro_angle(struct raw_data, float, struct initial_data);

/*	Calculates angle using only Accelerometer	*/
float calc_acce_angle(struct raw_data, struct initial_data);

/*	Calculates angle using Gyro data and Acce Data	*/
float complementary_filter(float, float, float);

/*	Returns appropriate angle depending on the filter selected	*/
void  calc_angle(float*, struct raw_data, struct initial_data, filter_type); 

/*TODO: See if you need the cpp gaurds
#ifdef __cplusplus
}
#endif
*/
#endif