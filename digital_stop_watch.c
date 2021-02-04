/*
 * digital_stop_watch.c
 *
 *  Created on: Feb 1, 2021
 *      Author: Belal Abdurazik
 *              Mini_Project1
 *
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


unsigned char tick = 0;
unsigned char second0 = 0;
unsigned char second1 = 0;
unsigned char minute0 = 0;
unsigned char minute1 = 0;
unsigned char hour0 = 0;
unsigned char hour1 = 0;


/*INT0(Falling edge using the internal pull up) --> reset the stop watch*/

void INT0_Init(void)
{
	DDRD &=(~(1<<PD2));  /*PD2 as input pin */
	PORTD |=(1<<PD2);    /*Enable internal pull up res*/
	GICR |=(1<<INT0);    /*Enable INT0*/
	MCUCR |=(1<<ISC01);  /*Trigger INT0 with the falling edge*/
}

ISR(INT0_vect)
{
 /*reset the stop watch*/
	second0=0;
	second1=0;
	minute0=0;
	minute1=0;
	hour0=0;
	hour1=0;
}
/**************************************************************************/
/*INT1(raising edge using the external pull down) --> pause the stop watch*/
void INT1_Init(void)
{
	DDRD &=(~(1<<PD3));   		     /*PD3 as input pin */
	GICR |=(1<<INT1);           	 /*Enable INT1*/
	MCUCR |=(1<<ISC11)|(1<<ISC10);   /*Trigger INT1 with the raising edge*/
}

ISR(INT1_vect)
{
	/*pause the stop watch*/
	TCCR1B &= ~(1<<CS10)&~(1<<CS11)&~(1<<CS12); /* No clock source (Timer/Counter stopped)*/

}

/**************************************************************************/
/*INT2(Falling edge using the internal pull up) --> resume the stop watch.*/
void INT2_Init(void)
{
	DDRB &=(~(1<<PB2));			     /*PB2 as input pin */
	GICR |=(1<<INT2);           	 /*Enable INT2*/
	MCUCSR &=(~(1<<ISC2));           /*Trigger INT2 with the falling edge*/
}

ISR(INT2_vect)
{
	/*resume the stop watch*/
	TCCR1B = (1<<WGM12) | (1<<CS10)|(1<<CS12);

}
/**************************************************************************/
void TIMER1_CTC_Init(void)
{
	TCCR1A = (1<<FOC1A);                       /*Non PWM*/
	TCCR1B = (1<<WGM12) | (1<<CS10)|(1<<CS12); /*prescaler=1024*/
	TCNT1 = 0;                                 /*Initial value*/
	OCR1A = 976; 							   /*Ft=Fcpu/N   Ft=10^6/1024 = 976*/
	TIMSK |= (1<<OCIE1A);
}
ISR(TIMER1_COMPA_vect)
{
	tick =1;
}

/**************************************************************************/
int main(void)
{
	DDRC |= 0x0F ;          /*First four pins in port c as output*/
	PORTC &= 0xF0 ;         /*Initial value*/

	DDRA |= 0x3F ;			/*First six pins in port a as output*/
	PORTA |= 0x3F ;

	SREG |=(1<<7) ;         /*Enable I-bit*/

	INT0_Init() ;
	INT1_Init() ;
	INT2_Init();
	TIMER1_CTC_Init() ;

	while(1)
	{
		if(tick == 1)
		{
			second0++;
			if(second0 > 9)
			{
				second0=0;
				second1++;
			}
			if(second1 > 5)
			{
				second1=0;
				minute0++;
			}
			if(minute0 > 9)
			{
				minute0=0;
				minute1++;
			}
			if(minute1 > 5)
			{
				minute1=0;
				hour0++;
			}
			if(hour0 > 9)
			{
				hour0=0;
				hour1++;
			}
			if(hour1 > 1)
			{
				if(hour0 > 3)
				{
					second0=0;
					second1=0;
					minute0=0;
					minute1=0;
					hour0=0;
					hour1=0;
				}
			}
			tick = 0 ;
		}
		else
		{
			/*First Digit in seconds*/
			PORTA = (1<<0);
			PORTC = second0;
			_delay_ms(5);

			/*second Digit in seconds*/
			PORTA = (1<<1);
			PORTC = second1;
			_delay_ms(5);

			/*First Digit in minutes*/
			PORTA = (1<<2);
			PORTC = minute0;
			_delay_ms(5);

			/*second Digit in minutes*/
			PORTA = (1<<3);
			PORTC = minute1;
			_delay_ms(5);

			/*First Digit in hours*/
			PORTA = (1<<4);
			PORTC = hour0;
			_delay_ms(5);

			/*second Digit in minutes*/
			PORTA = (1<<5);
			PORTC = hour1;
			_delay_ms(5);
		}
	}
}
