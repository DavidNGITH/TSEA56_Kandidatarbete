/*
 * manueldrive_2.c
 *
 * Created: 2023-04-14 11:46:12
 *  Author: erida646
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include <compat/twi.h>

#define I2C_BUFF_SIZE 2

volatile uint8_t I2C_read_buffer[I2C_BUFF_SIZE];
volatile uint8_t typedata_recieved = 0;
volatile uint8_t valuebyte = 0;

void PWM_pwr()
{
	//initiering
	
	DDRD = (1<<PD5); //sätter pin5 som output 
	TCCR1A =  (1<<WGM11) | (1<<COM1A1) ;
	TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS11); //fast PWM prescaler 8
	ICR1=1000;
	OCR1A = 0; //% av motorn krqaft
}
	
void servo()
{
	DDRB= (1<<PB6);
	TCCR3A =  (1<<WGM31) | (1<<COM3A1) ;
	TCCR3B = (1<<WGM32) | (1<<WGM33) | (1<<CS31);
	
	ICR3=40000;
	OCR3A =3035; // ligger mellan 2023(max vänster) - 4046(max höger) 3035(raktfram)
		
}

void breaking()
{
	DDRD |= (1<<DDD4); //sätter pin4 som output
	PORTD |= (0<<PD4);
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
	breaking();
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
	PORTA = TWSR; //Debugging


	uint8_t status = TWSR & 0xF8;
	static uint8_t bytecounter = 0;
	switch (status) {
		case TW_SR_SLA_ACK:
		bytecounter = 0;
			break;
		case TW_SR_DATA_ACK:
			if (bytecounter == 0)
			{
				typedata_recieved = TWDR;
			}
			else if (bytecounter == 1)
			{
				valuebyte = TWDR;
			}
			bytecounter++;
			break;

		case TW_SR_STOP:
			bytecounter = 0;
			if (typedata_recieved == 0)
			{
				OCR1A = valuebyte * 4;
			}
			else if (typedata_recieved == 1)
			{
				OCR3A = 2023 + (valuebyte * 20);
			}
			else if (typedata_recieved == 2)
			{
				PORTD |= (valuebyte<<PD4);
			}
				
			
			/*switch(typedata_recieved){
				case 0:
					OCR1A = TWDR * 4;
					break;
				case 1:
					OCR3A = 2023 + (TWDR * 20);
					break;
				default:
					break;}*/
			break;
		default:
			break;
		
	}
	TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
}
