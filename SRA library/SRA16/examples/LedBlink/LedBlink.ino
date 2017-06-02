// An Introductory Code to make LEDs on the SRA-Board blink in a pattern
// By Society Of Robotics And Automation, VJTI
// May 20, 2017

#include<sra16.h>   //  We include the sra16.h file 
                    //  So that we can use the functions already defined in it

void setup() {
  // put your setup code here, to run once:

  DDRC |= 0b11111111;   //  Declaring all pins of Port C as OUTPUT pins 
                        //  DDRC |= 0xFF; would also do the same
                        //  The LED pack is on Port C of the SRA Board
}

void loop() {
  // put your main code here, to run repeatedly:

  //  Calling the led_blink() function 
  led_blink(); 
}

//  Definition of our blink pattern in the function below
//  This function (led_blink()) when called executes the lines of code which are enclosed by the braces (i.e. "{", "}")
void led_blink()
{
  LED = 0b0000000;
  _delay_ms(100);
  LED = 0b10000000;
  _delay_ms(100);
  LED = 0b11000000;
  _delay_ms(100);
  LED = 0b11100000;
  _delay_ms(100);
  LED = 0b11100000;
  _delay_ms(100);
  LED = 0b11110000;
  _delay_ms(100);
  LED = 0b11111000;
  _delay_ms(100);
  LED = 0b11111100;
  _delay_ms(100);
  LED = 0b11111110;
  _delay_ms(100);
  LED = 0b11111111;
  _delay_ms(100);
}
