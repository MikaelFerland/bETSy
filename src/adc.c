#include "adc.h"

void ADC_Init(void)
{
	// ADC enable - Free run enable - 1/128 prescaler set  
	ADCSRA = ( (1 << ADEN) | (1 << ADATE) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS0) | (1 << ADPS1));
	
	ADMUX = 0;

	ADCSRA |= (1 << ADIE);

	
}
