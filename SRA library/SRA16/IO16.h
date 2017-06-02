

/* io_config .h */
#ifndef IO16_H
#define IO16_H

#include <stdint.h>
#include <util/delay.h>
#include <avr/io.h>

#ifndef F_CPU
	#define F_CPU 16000000UL
#endif



//BASIC I/O FILE SETTINGS


#define INPUT 0
#define OUTPUT 1

typedef enum _BOOL { FALSE = 0, TRUE } BOOL;

typedef struct
{
  unsigned int bit0:1;
  unsigned int bit1:1;
  unsigned int bit2:1;
  unsigned int bit3:1;
  unsigned int bit4:1;
  unsigned int bit5:1;
  unsigned int bit6:1;
  unsigned int bit7:1;
} _io_reg; 


#define REGISTER_BIT(rg,bt) ((volatile _io_reg*)&rg)->bit##bt




/** I N C L U D E S **/


//SENSOR NUMBER
#define sensor_num 4	//change this value for any other sensor

//MOTOR VALUES

#define MOTOR1A   	REGISTER_BIT(PORTC,4)
#define MOTOR1B    	REGISTER_BIT(PORTC,5)
#define MOTOR2A    	REGISTER_BIT(PORTC,6)
#define MOTOR2B    	REGISTER_BIT(PORTC,7)


//MOTOR DIRECTIONS

#define MOTOR1A_DIR   	REGISTER_BIT(DDRC,4)
#define MOTOR1B_DIR     REGISTER_BIT(DDRC,5)
#define MOTOR2A_DIR     REGISTER_BIT(DDRC,6)
#define MOTOR2B_DIR     REGISTER_BIT(DDRC,7)


//ADC 	DIRECTIONS

#define ADC			PORTA
#define ADC_DIR 	DDRA


//PWM pins

//PWM0

#define PWM0_DIR  	REGISTER_BIT(DDRB,3)
#define PWM0  		REGISTER_BIT(PORTB,3)

//PWM1A

#define PWM1A_DIR  	REGISTER_BIT(DDRD,4)
#define PWM1A 		REGISTER_BIT(PORTD,4)

//PWM1B

#define PWM1B_DIR  	REGISTER_BIT(DDRD,5)
#define PWM1B 		REGISTER_BIT(PORTD,5)

//PWM2

#define PWM2_DIR  	REGISTER_BIT(DDRD,7)
#define PWM2 		REGISTER_BIT(PORTD,7)

//DOTBAR LED

#define LED			PORTC
#define LED_DIR 	DDRC


//LCD CONNECTIONS

#define LCD_DATA B	//Port PC0-PC3 are connected to D4-D7

#define LCD_E B		//Enable OR strobe signal
#define LCD_E_POS	PB7	//Position of enable in above port

#define LCD_RS B	
#define LCD_RS_POS 	PB5

#define LCD_RW B
#define LCD_RW_POS 	PB6

//FUNCTION PROTOTYPES
void port_init(void);
void pwm1_init(void);
void adc_init(void);
unsigned char adc_start(unsigned char channel);
void delay_sec(int x);
void delay_milisec(int n);
void delay_microsec(int n);
void check_sensors(void);
void calibrate_black(void);
void calibrate_white(void);
void set_threshold(void);
void flick (void);
void set_pwm1a(int a);
void set_pwm1b(int b);
void lcd_byte(uint8_t c,uint8_t isdata);
void lcd_busy_loop(void);
void lcd_init(uint8_t style);
void lcd_write_string(const char *msg);
void lcd_write_int(int val,unsigned int field_length);
void lcd_goto_xy(uint8_t x,uint8_t y);
void lcd_write_string_xy(int x,int y,char *msg);
void lcd_write_int_xy(int x,int y,int val,int fl);
void usart_init(void);
void usart_transmit_char( unsigned char data );
void usart_transmit_string(char *msg );
void usart_transmit_newline(void);
unsigned char usart_receive_char(void);
#endif //IO_H
