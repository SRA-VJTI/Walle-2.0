//	MPU LIBRARY FOR WALL-E 2.0
//	By Society Of Robotics And Automation

#include <MPU.h>
#include <Arduino.h>

#define RAD_TO_DEG 	57.2975                        									// 180 / Pi
#define MPU_ADDR 	0x68

void start_mpu(void)
{
	Wire.begin();
	Wire.beginTransmission(MPU_ADDR);												// START signal
	Wire.write(0x6B);           													// PWR_MGMT_1 register
	Wire.write(0);              													// set to zero (wakes up the MPU-6050)
	Wire.endTransmission(true);														// STOP signal
}

void read_raw_values_mpu(struct raw_data* raw_val)
{
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x3B);  																// Starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);													// REPEATED START signal
	Wire.requestFrom(MPU_ADDR,14,true);  											// request a total of 14 registers
	
	raw_val->raw_acce_x	=	Wire.read()	<<	8	|	Wire.read();  					// 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
	raw_val->raw_acce_y	=	Wire.read()	<<	8	|	Wire.read();  					// 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	raw_val->raw_acce_z	=	Wire.read()	<<	8	|	Wire.read();  					// 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	raw_val->temp		=	Wire.read()	<<	8	|	Wire.read();  					// 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	raw_val->raw_gyro_x	=	Wire.read()	<<	8	|	Wire.read();  					// 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	raw_val->raw_gyro_y	=	Wire.read()	<<	8	|	Wire.read();  					// 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	raw_val->raw_gyro_z	=	Wire.read()	<<	8	|	Wire.read();  					// 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void calibrate_mpu(struct initial_data* init_data)
{
	init_data->gyro_reading	= 0;													// Initialize everything to zero
	init_data->acce_angle	= 0;
    static long 	accX	= 0;
	static long 	accZ	= 0;
    static struct	raw_data raw_values;
	static long		raw_acce_xz;
	
    for(int i = 0; i < 2000; i++) {													// Calculate the average initial reading over 2000 readings
        read_raw_values_mpu(&raw_values);
        accX += raw_values.raw_acce_x;
        accZ += raw_values.raw_acce_z;
        init_data->gyro_reading += raw_values.raw_gyro_y;
    }

    accX = accX / 2000;
    accZ = accZ / 2000;
	
    raw_acce_xz = sqrt((accX * accX) + (accZ * accZ));

    init_data->acce_angle	= asin(((float)accX) / ((float)raw_acce_xz)) * RAD_TO_DEG; 
    init_data->gyro_reading	= init_data->gyro_reading	/ 2000;
}