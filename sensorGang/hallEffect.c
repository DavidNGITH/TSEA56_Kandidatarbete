/*
 * Halleffect.c
 *
 * Created: 2023-03-29 17:36:18
 * Author : filed881
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>

static volatile int i = 0;

static volatile int pulseCnt = 0;


void timer1_init()
{
	TCCR1B = (1<<CS12) | (1<<CS10) | (1<<WGM12);// Startar timer prescaler 1024
	//TCCR1A = 1<<WGM11; //CTC mode
	
	TCNT1 = 0;

	OCR1A = 15625; //Hur ofta avbrott
	
	TIMSK1 = 1 << OCIE1A; //Aktiverar avbrott
}

void port_init()
{
	//DDRD = 1 << DDD5; //Sätter port PD5 till output
	
	DDRA = 0xFF; //Sätter port A till output
	
	DDRD = 0 << DDD3; //Sätter port PD3 till input (INT1)
}

void int1_init()
{
	EIMSK |= 1 << INT1; //Enable INT1 interrupt
	
	EICRA = (1<<ISC11) | (1<<ISC10); //Interrupt on rising edge
}

int main(void)
{	
	port_init();
	
	int1_init();
	
	timer1_init();
	
	sei();
	
    /* Replace with your application code */
    while (1) 
    {
		printf(1);
    }
}

ISR(TIMER1_COMPA_vect)
{
	PORTA = pulseCnt;
	
	pulseCnt = 0;
	
	//TCNT1=0;
}

ISR(INT1_vect)
{
	pulseCnt++;
}
