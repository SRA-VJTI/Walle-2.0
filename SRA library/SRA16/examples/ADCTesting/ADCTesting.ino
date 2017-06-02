// A Code to demonstrate Analog to Digital Conversion (ADC)
// By Society Of Robotics And Automation, VJTI
// May 20, 2017

#include<sra16.h>   //  We include the sra16.h file 
                    //  So that we can use the functions already defined in it

int sensorData;     //  A variable of type int (which is 16 bit long)
                    //  to store the data returned by the sensor
                    
void setup() {
  // put your setup code here, to run once:

  adc_init();       //  Initializes the ADC functionality
                    //  Analog to Digital Converters (ADCs) are required to 
                    //  express analog voltages as digital values (in this case as a number b/w 0 and 255)
                    //  because only digital values can worked upon and manipulated in micro-controllers (and most computers as well)
}

void loop() {
  // put your main code here, to run repeatedly:

  sensorData =  adc_start(0);     //  The function returns the value returned 
                                  //  by the sensor connected to pin 0 of port A (i.e. A0)
                                  //  This value is stored in the sensorData variable

  Serial.println(sensorData);     //  Prints the variable sensorData onto the Serial Monitor 
}
