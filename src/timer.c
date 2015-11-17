/*
    Title:    timer.c
    Authors:  Mikael Ferland, Joël Brisson
    Date:     4/10/2015
    Purpose:  Initialysation, writing and reading for TWI between master and slaves, using i2c protocol.
    Software: AVR-GCC to compile
    Hardware: ATMega32 on STK500 board
    Note:     0xFE and 0xFF is reserved by the system for restart and stop condition.
*/

#include "../include/timer.h"

void timer_Init()
{
/*
	The PWM frequency for the output can be calculated by the following equation:	
	focnPWM=Fcpu/(N*256)
		N is the prescaler factor
*/
	
	//Fast PWM mode 10bits with /8 prescale
	outp((1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(0<<FOC1A)|(0<<FOC1B)|(1<<WGM11)|(0<<WGM10),TCCR1A);
	outp((0<<ICNC1)|(0<<ICES1)|(1<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10),TCCR1B);

	//Set TOP Value to get 5ms with /8 prescale
	ICR1 = 0x270F;

	//Enable Timer1 Overflow Interrup
	outp((1<<TOIE1),TIMSK);
}
