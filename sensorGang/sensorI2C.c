/*
 * IC2.c
 *
 * Created: 2023-03-31 10:15:58
 * Author : filed881
 */ 
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include <compat/twi.h>
#include <stdlib.h>




#define I2C_BUFF_SIZE 2
#define TW_MT_DATA_ACK     0x28

volatile uint8_t I2C_read_buffer[I2C_BUFF_SIZE];

static volatile int pulseCnt = 0; //Ska skickas, antal pulser från halleffect
static volatile int i = 0;
static volatile int pulse = 0; //Ska skickas pulsbredden från ultrasonic
static volatile int ultraCnt = 0;

uint8_t readTWDR = 16;


void i2c_init()
{
	TWAR = 0xD4; //Adress for the slave which is at 0x6a
	
	TWCR = (1<<TWEN)|(1<<TWEA)|(1<<TWIE);
	
	//TWCR = 0xC5;
}

void port_init()
{
	//ULTRALJUD
	DDRD |= (1 << DDD5); //PD5 Output
	DDRD |= (0 << DDD2); //PD2 (INT0) Input
	
	//HALLEFFECT
	DDRD |= (0 << DDD3); //PD3 (INT1) Input
	
	
	//TESTNING
	DDRA = 0xFF; //Port A Output
	
}

//TIMER TILL ULTRASONIC
void timer0_init()
{
	TCCR0A = (1<<WGM01); //CTC mode
	TCCR0B = (1<<CS02) | (1<<CS00); //Prescaler 1024
	
	OCR0A = 255; //Hur ofta vi vill generera avbrott på A
	
	OCR0B = 0; //Hur ofta vi vill generera avbrott på A
	
	TIMSK0 = (1<<OCIE0A) | (1<<OCIE0B); //Enable interrupt på A och B
}

//TIMER TILL HALLEFFECT
void timer1_init()
{
	TCCR1B = (1<<CS12) | (1<<CS10) | (1<<WGM12);// Startar timer CTC prescaler 1024
	
	TCNT1 = 0;

	OCR1A = 15625; //Hur ofta avbrott
	
	TIMSK1 = 1 << OCIE1A; //Aktiverar avbrott på A
}

//INTERRUPT TILL HALLEFFECT
void int1_init()
{
	EIMSK |= 1 << INT1; //Enable INT1 interrupt
	
	EICRA |= (1<<ISC11) | (1<<ISC10); //Interrupt on rising edge
}

//INTERRUPT TILL ULTRASONIC
void int0_init()
{
	EIMSK |= 1 << INT0; //Enable av INT0
	
	EICRA |= (1 << ISC00); //Any edge interrupt
}

int main(void)
{
	port_init();
	
	int1_init();
	
	int0_init();
	
	timer1_init();
	
	timer0_init();
	
	i2c_init();
	
	sei();
	
	while (1)
	{
		printf("1");
		
	}
}

//INTERRUPTRUTIN HALLEFFECT
ISR(TIMER1_COMPA_vect)
{
	//PORTA = pulseCnt;
	
	I2C_read_buffer[1] = pulseCnt;
	
	pulseCnt = 0;
	
	//TCNT1=0;
}

//INTERRUPTRUTIN ULTRASONIC
ISR(TIMER0_COMPA_vect)
{
	if (ultraCnt == 6){
		PORTD = 1<<PORTD5;
		ultraCnt = 0;
	}
	else{
		ultraCnt++;
		}
}

//INTERRUPTRUTIN ULTRASONIC
ISR(TIMER0_COMPB_vect)
{
	PORTD = 0<<PORTD5;
}

//INTERRUPTRUTIN INT1 HALLEFFECT
ISR(INT1_vect)
{
	pulseCnt++;
}

//INTERRUPTRUTIN INT0 ULTRASONIC
ISR(INT0_vect)
{
	if(i) //Disable counter
	{
		TCCR3B = 0;  //Stänger av timer
		
		pulse = TCNT3; //Sparar värdet från timern
		
		if (pulse > 250)
		{
			pulse = 250;
		}
		
		I2C_read_buffer[0] = pulse;
		
		//PORTA = TCNT3;
		
		TCNT3 = 0; //Nollställer timer
		
		i = 0;
	}
	
	else
	{
		TCCR3B = (1 << CS32)|(1 << CS30); //Prescaler 1028
		
		i = 1;
	}
}

ISR(TWI_vect)
{
	PORTA = TWSR;
	uint8_t status = TWSR & 0xF8;
	switch (status) {
		case TW_SR_DATA_ACK:
			readTWDR = TWDR;
			break;
		case TW_ST_SLA_ACK:
			switch(readTWDR){
				case 0:
					TWDR = I2C_read_buffer[0];
					break;
				case 1:
					TWDR = I2C_read_buffer[1];
					break;
				case 2:
					TWDR = 2;
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
