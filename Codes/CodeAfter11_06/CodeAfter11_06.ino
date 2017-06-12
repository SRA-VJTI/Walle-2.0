	/**
	Self-Balancing Bot
	Demonstrates the working of Balancing Algorithm. 

	For more information visit http://sra.vjti.info/

	modified 10 June, 2017
	by Society Of Robotics And Automation, VJTI.
  **/

#define no_serial 1

#define calib_mpu 1

#include <MPU.h>
#include <FILTER.h>
#include <SRA16.h>
#include <BBM.h>

//PID Params
#define KP_CONST 1
#define KI_CONST 0
#define KD_CONST 0.01

#define Kp 27
#define Kd 0.1
#define Ki 0.8

//Filter Params
struct raw_data 	raw_values;
struct initial_data	init_data;
struct initial_data new_init_data;

float angleY = 0;
float temp = 0;
short flag = 0;

int i = 0;
int t= 0;
struct pid_controller{
	float kp;
	float ki;
	float kd;
	float error;
	float diff_in_error;
  float correction;
  float prop;
	float prev_error;
	long  cumulative_error;
};

pid_controller balancing_pid_ctrl = {Kp, Ki, Kd, 0, 0, 0, 0, 0};//kp 20-25, kd 0.3 //kp 14, kd 0.08
																

void setup()
{
	/*	Begin Serial Communication between SRA-Board and CP2102	*/
  #if no_serial
	Serial.begin(9600);
  #endif

	/*	Belongs to the MPU module	
		Wakes up the MPU with particular settings	*/
	start_mpu();                                                      

	/*	Belongs to the MPU module
		Gets the initial values of the angular velocity using the gyroscope and
		angle using the accelerometer, with respect to which new angles will be calculated	*/
  //#if calib_mpu
	  calibrate_mpu(&init_data);         
  //#endif
  
	/*	Initializes the motors and PWM (PWM1)	*/
	bot_motion_init(); 

	/*	Belongs to the sra16 module
		Initializes switches on the SRA-Board	*/
	switch_init();                  

	//  HARD CODED VALUES
 
  //#//if !calib_mpu
	//init_data.acce_angle	= 4.21;//4.61//4.7//10.5//5.19 // 12.34
	//init_data.gyro_reading	= 116; //179 // 				//137
  //#endif
  
	/*	Used for tuning values of kp, and kd using the available Potentiometers	*/
//	tune_pid_pot(&balancing_pid_ctrl);  
}

void loop()
{
	/*  This function belongs to the MPU modulue,
		it reads raw values of the accelerometer and gyroscope's registers from the MPU	*/	
	read_raw_values_mpu(&raw_values); 
	
	/*  This function belongs to the Filter Module and it calculates the current angle about the Y-Axis.
		This marks the connection between the MPU module and the Filter module	*/
	calc_angle(&angleY, raw_values, init_data, COMPLEMENTARY);                
	
	/*	This function calculates the error using the angle about Y-Axis i.e. angleY
		The value of angleY is obtained from the calc angle function
		This marks the connection between the Filter module and the PID controller	*/
	calc_error_using_pid(&balancing_pid_ctrl, angleY);    
	
	/*	This function gives instructions to the motor to balance according
		to the error obtained from the previous calc_error_using_pid() function
		This marks the connection between the PID controller anf the Balancing algorithm	*/
	balance(balancing_pid_ctrl.correction); 
  /*
	if(pressed_switch2()){
		flag = 1;	
	}

	if(flag){
		PORTC |= 0b00000100;
		move(FORWARD, 20, &balancing_pid_ctrl.error);	
	}
*/
 #if no_serial
	print_data();
 #endif
// if(pressed_switch0()){
//  calibrate_mpu(&init_data);
// }
/* if(pressed_switch3()) {
   balancing_pid_ctrl.ki -= 0.1;
   if(balancing_pid_ctrl.kp  < 0){
    balancing_pid_ctrl.ki = 0;
   }
 }
 if(pressed_switch2()) {
   balancing_pid_ctrl.ki += 0.1;
 }*/
 if(pressed_switch2()){
  t=millis();
  temp = init_data.acce_angle;
  //move(FORWARD, 10, &init_data);
  init_data.acce_angle += 2;
  PORTC |= 0b00000100;
  flag = 1;
 }
// if(pressed_switch3()){
//  init_data.acce_angle -= 2;
// }
 i = millis()-t;
 if (i>1000&&flag==1){
  init_data.acce_angle -= 2;
    PORTC |= 0b00001000;
    flag = 2;
 }
}

void print_data()
{
//    Serial.print(raw_values.raw_acce_x);Serial.print("\t");
//    Serial.print(raw_values.raw_acce_y);Serial.print("\t");
//    Serial.print(raw_values.raw_acce_z);Serial.print("\t");
//    Serial.print(raw_values.temp);Serial.print("\t");
//    Serial.print(raw_values.raw_gyro_x);Serial.print("\t");
//    Serial.print(raw_values.raw_gyro_y);Serial.print("\t");
//    Serial.print(raw_values.raw_gyro_z);Serial.print("\t");
    Serial.print(angleY);Serial.print("\t");
//    Serial.print(correction);Serial.print("\t");
	Serial.print(init_data.acce_angle);Serial.print("\t");
	Serial.print(init_data.gyro_reading);Serial.print("\t");
//    Serial.print(i);Serial.print("\t");
//     Serial.print(t);Serial.print("\t");
//    Serial.print(balancing_pid_ctrl.correction);
//    Serial.print("\t");
//    Serial.print(angleY);
//    Serial.print("\t");
    
//    Serial.print(balancing_pid_ctrl.diff_in_error);
//    Serial.print("\t");
//    Serial.print(balancing_pid_ctrl.cumulative_error);
//    Serial.print("\t");
//    Serial.print(balancing_pid_ctrl.error);
//    Serial.print("\t");
//    Serial.print(balancing_pid_ctrl.prev_error);
//    Serial.print("\t");
	  Serial.println();
}

//	SETTING MOTOR SPEED
void left_motorspeed(int a)
{
	  set_pwm1a(a); 
}
void right_motorspeed(int b)
{
    set_pwm1b(b); 
}

//	CALCULATION OF ERROR USING PID
void calc_error_using_pid(struct pid_controller* pid, float theta)
{
    pid->error             = theta;                                                  //  WE PLACE THE MPU SUCH THAT THE BOT TILTS ABOUT Y-AXIS
    pid->diff_in_error     = pid->error - pid->prev_error;
    if(pid->error < 10 && pid->error > -10)
      pid->cumulative_error = 0;
    if(pid->cumulative_error < 5000 && pid->cumulative_error > -5000)
      pid->cumulative_error += pid->error;
    pid->prev_error        = pid->error;
    pid->correction        = pid->kp * pid->error + pid->ki * pid->cumulative_error - pid->kd * pid->diff_in_error;
}

//	BALANCING FUNCTION
void balance(float correction)
{
  long abs_correction = abs(correction);
	abs_correction += 110;
	
	if(correction > 0) {
		bot_backward();
	}
	else if(correction < 0) {
		bot_forward();
	}
	else bot_brake();

	abs_correction = constrain(abs_correction, 125, 399);
	  left_motorspeed(abs_correction);
    right_motorspeed(abs_correction);
}

//	PID TUNING WITH POT
void tune_pid_pot(struct pid_controller* pid)
{
	//Serial.println("TUNING STARTED");
	short done_tuning = 0;

	while(!done_tuning) { 
		pid->kp = adc_start(4) * KP_CONST;
		pid->kd = adc_start(5) * KD_CONST;

		//Serial.print(done_tuning);	Serial.print("\t");
		//Serial.print("kp:\t");		Serial.print(pid->kp);
		//Serial.print("\tkd:\t");	Serial.print(pid->kd, 4);
		//Serial.println();

		if(pressed_switch0()) {
			done_tuning = 1;
		}
	}
}

