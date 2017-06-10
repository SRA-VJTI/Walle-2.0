//	BALANCED-BOT-MOTION (BBM) LIBRARY FOR WALL-E 2.0
//	By Society Of Robotics And Automation

#include <BBM.h>
#include <MPU.h>

void move(dir direction, short speed, struct initial_data* init_data)
{
	float constant 	= 0.4;												//	constant to multiply the speed with
	
	if(direction == FORWARD)											//	If the direction is FORWARD
		init_data->acce_angle	+= speed * constant;					//	move the set-point forward
	else if(direction == BACKWARD)										//	If the direction is BACKWARD
		init_data->acce_angle	-= speed * constant;					//	move the set-point backward
}

void move(dir direction, short speed, float* error)
{
	float constant 	= 5;												//	constant to multiply the speed with
	
	if(direction == FORWARD)											//	If the direction is FORWARD
		*error	+= speed * constant;									//	forward error bias
	else if(direction == BACKWARD)										//	If the direction is BACKWARD
		*error	-= speed * constant;									//	backward error bias
}