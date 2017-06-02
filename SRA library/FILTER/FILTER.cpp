//	FILTER LIBRARY FOR WALL-E 2.0
//	By Society Of Robotics And Automation

#include <Filter.h>
#include <Arduino.h>

//	A function to calcuulate the angular velocity about the required axis using Gyroscope Data
float calc_angular_velocity(struct raw_data raw_values, struct initial_data init_data)
{
	return ((raw_values.raw_gyro_y - init_data.gyro_reading) / 131);                     // FS_SEL = 0, so 131 raw reading corrusponds to 1 deg / s BY default FS_SEL = 0
}

//	A function to calculate angle using only Gyroscope data
float calc_gyro_angle(struct raw_data raw_values, float prev_angle, struct initial_data init_data)
{
	long current_time 		= millis();
	int time_diff			= current_time - prev_time;
	prev_time        		= current_time;
	return prev_angle + calc_angular_velocity(raw_values, init_data) * time_diff * 0.001;
}

//	A function to calculate angle using only Accelerometer data
float calc_acce_angle(struct raw_data raw_values, struct initial_data init_data)
{
	long accX = (long)raw_values.raw_acce_x;
    long accZ = (long)raw_values.raw_acce_z;

    long raw_acce_xz         = sqrt((accX * accX) + (accZ * accZ));
    float current_acce_angle = asin(((float)accX) / ((float)raw_acce_xz)) * RAD_TO_DEG; 
	
    return current_acce_angle - init_data.acce_angle;
}

//	A function to calculate the angle using both Gyroscope and Accelerometer data along with Complementary Filter
float complementary_filter(float angular_velocity, float acce_angle, float prev_angle, float gyro_wt)
{
	long current_time 		= millis();
	int time_diff			= current_time - prev_time;
	prev_time        		= current_time;
	return (gyro_wt * (prev_angle + angular_velocity * time_diff * 0.001) + (1 - gyro_wt) * acce_angle);
}

// A function that returns angles Depending on the Filter selected
void calc_angle(float* angle, struct raw_data raw_values, struct initial_data init_data, filter_type filter_chosen)
{
	if(filter_chosen == NONE) {
		*angle	= calc_gyro_angle(raw_values, *angle, init_data);
	}
	
	else if(filter_chosen == COMPLEMENTARY)	{
		float angular_velocity	= calc_angular_velocity(raw_values, init_data);
		float acce_angle		= calc_acce_angle(raw_values, init_data);
		*angle					= complementary_filter(angular_velocity, acce_angle, *angle, 0.98);
	}

	else if(filter_chosen == KALMAN) {
	//TODO: Add this ? 
	}
}