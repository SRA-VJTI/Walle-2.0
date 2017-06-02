// A Code to demonstrate Pulse Width Modulation (PWM)
// By Society Of Robotics And Automation, VJTI
// May 20, 2017

#include<sra16.h>   //  We include the sra16.h file 
                    //  So that we can use the functions already defined in it

void setup() {
  // put your setup code here, to run once:

  bot_motion_init();    //  Declares the pins of the ATmega16 corrusponding to the motors as OUTPUT
  
  pwm1_init();          //  Initializes the PWM functionality using Timer1
                        //  PWM stands for Pulse Width Modulation 
                        //  without which it is not possible to control motors with variable speed
                        //  Timer1 is one of the timers of the ATmega16 
                        //  Timers allow accurate program execution timing (event management),
                        //  wave generation, and signal timing measurement. 
                        
  switch_init();        //  Initializes the Switches by declaring the required pins on D port as INPUT
}

void loop() {
  // put your main code here, to run repeatedly:
  
  bot_forward();        //  Sets the direction of the motor terminals such that the bot moves forward
  
  set_left_motorspeed(350);
  set_right_motorspeed(350);
  _delay_ms(1000);
  
  set_left_motorspeed(300);
  set_right_motorspeed(300);
  _delay_ms(1000);
  
  set_left_motorspeed(250);
  set_right_motorspeed(250);
  _delay_ms(1000);
  
  set_left_motorspeed(200);
  set_right_motorspeed(200);
  _delay_ms(1000);
}

//  Funtions to set the speed (PWM) of the Left and Right motors
void set_left_motorspeed(int speed){
    set_pwm1a(speed);   //  Sets OCR1A to speed
                        //  An OCRx Register (Output Compare Register)
                        //  compared with the Timer/Counter (TCNTx which is a register belonging to Timerx,
                        //  which counts numbers in sequential order (E.g. 0, 1, 2 ...) )
                        //  value at all time. The result of the compare can be used to
                        //  generate a PWM or variable frequency
  }

void set_right_motorspeed(int speed){
    set_pwm1b(speed);
  }
