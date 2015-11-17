/*
    Title:    adc.c
    Authors:  Mikael Ferland, Joël Brisson
    Date:     24/10/2015
    Purpose:  Initialysation the Analogue to Digital Converter.
    Software: AVR-GCC to compile
    Hardware: ATMega32 on STK500 board
    Note:
*/

#include "adc.h"

void ADC_Init(void)
{
	//Bit 7 – ADEN: ADC Enable
	//	Writing this bit to one enables the ADC. By writing it to zero, the ADC is turned off. Turning
	//	the ADC off while a conversion is in progress, will terminate this conversion.
	
	//Bit 6 – ADSC: ADC Start Conversion
	//	In Free Running mode, write this bit to one to start the first conversion. The first conversion after
	//	ADSC has been written after the ADC has been enabled, or if ADSC is written at the
	//	same time as the ADC is enabled, will take 25 ADC clock cycles instead of the normal 13.
	
	//Bit 5 – ADATE: ADC Auto Trigger Enable
	//	When this bit is written to one, Auto Triggering of the ADC is enabled. The ADC will start
	//	a conversion on a positive edge of the selected trigger signal. The trigger source is
	//	selected by setting the ADC Trigger Select bits, ADTS in SFIOR.
	
	// ADPS2 | ADPS | ADPS0 | Division |
	//       |      |       | Factor   |
	//   0   |  0   |   0   |  2   	   |
	//   0   |  0   |   1   |  2       |
	//   0   |  1   |   0   |  4	   |
	//   0   |  1   |   1   |  8	   |
	//   1   |  0   |   0   |  16      |
	//   1   |  0   |   1   |  32      |
	//   1   |  1   |   0   |  64      |
	//   1   |  1   |   1   |  128     |
	
	// ADC enable - Free run enable - 1/128 prescaler set  
	ADCSRA = ( (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADPS2) | (1 << ADPS0) | (1 << ADPS1));
	
	//Start reading the left motor
	ADMUX = 0;

	ADCSRA |= (1 << ADIE);

	
}
