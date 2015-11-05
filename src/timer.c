#include "../include/timer.h"

void timer_Init()
{
	//Fast PWM mode 10bits with /8 prescale
	outp((1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(0<<FOC1A)|(0<<FOC1B)|(1<<WGM11)|(0<<WGM10),TCCR1A);
	outp((0<<ICNC1)|(0<<ICES1)|(1<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10),TCCR1B);

	//TOP Value to get 5ms with /8 prescale
	ICR1 = 0x270F;

	//Enable Timer1 Overflow Interrup
	outp((1<<TOIE1),TIMSK);
}
