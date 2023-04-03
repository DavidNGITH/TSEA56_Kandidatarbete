/*
 * test_styr.cpp
 *
 * Created: 2023-03-29 14:47:09
 *  Author: malst829
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include <compat/twi.h>

#define I2C_BUFF_SIZE 2

volatile uint8_t I2C_read_buffer[I2C_BUFF_SIZE];


void PWM_pwr()
{
	//initiering
	
	DDRD = (1<<PD5); //sätter pin5 som output 
	TCCR1A =  (1<<WGM11) | (1<<COM1A1) ;
	TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS11); //fast PWM prescaler 8
	ICR1=1000;
	OCR1A =1000; //% av motorn krqaft
}
	
void servo()
{
	DDRB= (1<<PB6);
	TCCR3A =  (1<<WGM31) | (1<<COM3A1) ;
	TCCR3B = (1<<WGM32) | (1<<WGM33) | (1<<CS31);
	
	ICR3=40000;
	OCR3A =2023; // ligger mellan 2023(max vänster) - 4046(max höger) 3035(raktfram)
		
}

void I2C_init()
{
	TWAR = 0x94; // Device’s Own Slave Address hexa 0x4a
	TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWEN) | (1<<TWINT);
	
}


ISR(ADC_vect)
{
	//External interrupt
}

int main(void)
{
	
	PWM_pwr();
	servo();
	I2C_init();
	sei(); //aktivera globala avbrott
    while(1)
    {	
        //OCR1A = 255;
    }
}

ISR(TWI_vect)
{
	//PORTA = TWSR; //Debugging
	uint8_t typedata_recieved = 0;
	uint8_t status = TWSR & 0xF8;
	
	switch (status) {
		case TW_SR_SLA_ACK:
			typedata_recieved = TWDR;
			break;
		case TW_SR_DATA_ACK:
			switch(typedata_recieved){
				case 0:
					OCR1A = TWDR * 4;
					break;
				case 1:
					OCR3A = 2023 + (TWDR * 20);
					break;
				default:
					break;}
			break;
		case TW_SR_STOP:
			break;
		default:
			break;
		
	}
	TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
}
