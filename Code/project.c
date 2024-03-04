/*
 * project.c
 *
 *  Created on: May 2, 2021
 *      Author: Ammar_Yasser
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
unsigned char tick=0;
unsigned char sec=0;
unsigned char min=0;
unsigned char hour=0;
void TIMER1_Init(void)
{
	TCCR1A = (1<<FOC1A); //FOC1A = 1 when non PWM is chosen
	TCCR1B = (1<<WGM12)|(1<<CS10)|(1<<CS12); //WGM12 = 1 to operate at compare mode And 1024 prescale
	SREG|=(1<<7);  // Global bit interrupt
	OCR1A = 976.5625;
	TCNT1 = 0;
	TIMSK|=(1<<OCIE1A); // Timer1 Compare match interrupt
}
ISR(TIMER1_COMPA_vect)
{
	sec++;
	if (sec==60)
	{
		sec=0;
		min++;
	}
	if (min==60)
	{
		sec=0;
		min=0;
		hour++;
	}
	if (hour==12)
	{
		sec=0;
		min=0;
		hour=0;
	}
}
void INT0_Init_Rest(void)
{
	DDRD &=~(1<<PD2);
	PORTD |=(1<<PD2);    //internal pull up
	MCUCR |=(1<<ISC01);  //falling edge
	GICR  |=(1<<INT0);
	SREG  |=(1<<7);
}
ISR(INT0_vect)
{
	sec=0; min=0; hour=0;
}
void INT1_Init_Pause(void)
{
	DDRD &=~ (1<<PD3);
	MCUCR |= (1<<ISC11)| (1<<ISC10); // rising edge
	GICR  |= (1<<INT1);
	SREG  |= (1<<7);
}
ISR(INT1_vect)
{
	TCCR1B &= ~(1<<CS10)&~(1<<CS11)&~(1<<CS12);
}
void INT2_Init_Resume(void)
{
	DDRB  &=~(1<<PB2);
	PORTB |= (1<<PB2);   // internal pull up
	MCUCR &=~(1<<ISC2);	// falling edge
	GICR  |=(1<<INT2);
	SREG  |=(1<<7);
}
ISR(INT2_vect)
{
	TCCR1B = (1<<WGM12)|(1<<CS10)|(1<<CS12);

}
int main()
{
	DDRA= 0xff; // 7segement enable pins to output
	PORTA=0xff;
	DDRC=0x0f; //  7segement data pins to output
	PORTC=0X00; // initial value off

	INT0_Init_Rest();
	INT1_Init_Pause();
	INT2_Init_Resume();
	TIMER1_Init();
	while(1)
		{
			PORTA = (1<<5);
			PORTC = sec % 10;
			_delay_ms(5);
			PORTA = (1<<4);
			PORTC = sec / 10;
			_delay_ms(5);
			PORTA = (1<<3);
			PORTC = min % 10;
			_delay_ms(5);
			PORTA = (1<<2);
			PORTC = min / 10;
			_delay_ms(5);
			PORTA = (1<<1);
			PORTC = hour % 10;
			_delay_ms(5);
			PORTA = (1<<0);
			PORTC = hour / 10;
			_delay_ms(5);
		}
}
