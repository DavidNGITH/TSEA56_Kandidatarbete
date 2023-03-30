/*
 * Sensors.c
 *
 * Created: 2023-03-30 13:31:42
 * Author : filed881
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

static volatile int pulseCnt = 0;
static volatile int i = 0;
static volatile int pulse = 0;


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
	
	OCR0B = 1; //Hur ofta vi vill generera avbrott på A
	
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
	
	sei();
	
	while (1)
	{
		printf(1);
	}
}

//INTERRUPTRUTIN HALLEFFECT
ISR(TIMER1_COMPA_vect)
{
	//PORTA = pulseCnt;
	
	pulseCnt = 0;
	
	//TCNT1=0;
}

//INTERRUPTRUTIN ULTRASONIC
ISR(TIMER0_COMPA_vect)
{
	PORTD = 1<<PORTD5;
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
		
		PORTA = TCNT3;
		
		TCNT3 = 0; //Nollställer timer
		
		i = 0;
	}
	
	else
	{
		TCCR3B = (1 << CS32)|(1 << CS30); //Prescaler 1028
		
		i = 1;
	}
}