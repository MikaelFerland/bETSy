/*
    Title:    uart.c
    Authors:  Mikael Ferland, Joël Brisson
    Date:     xx/09/2015
    Purpose:  A collection of function to use UART originally provide for the course ELE542
    	      modified to match specifications.
    Software: AVR-GCC to compile
    Hardware: ATMega32 on STK500 board
    Note:     We added a state machine to set properly all UART bytes.
*/

#include "../include/uart.h"

/* UART global variables */
volatile u08   UART_Ready;
volatile u08   UART_ReceivedChar;
         u08   UART_RxChar;
         u08*  pUART_Buffer;
	 
volatile u08   ECHO = 0;
volatile u08   COMMAND = 0;
volatile u08   SPEED = 0;
volatile u08   ANGLE = 0;
volatile u08   STATE = 0;
volatile u08   PACKET_READY = 0;

/* end-of-line string = 'Line End' + 'Line Feed' character */
prog_char UART_pszEndOfLine[3] = {0x0d,0x0a,0};

/* UART Transmit Complete Interrupt Function */
SIGNAL(SIG_UART_TRANS)      
{
    /* Test if a string is being sent */
    if (pUART_Buffer!=0)
    {
        /* Go to next character in string */
        pUART_Buffer++;
        /* Test if the end of string has been reached */
        if (*pUART_Buffer==0)
        {
            /* String has been sent */
            pUART_Buffer = 0;
            /* Indicate that the UART is now ready to send */
            UART_Ready   = 1;
            return;
        }
        /* Send next character in string */
        outp( *pUART_Buffer, UDR );
        return;
    }
    /* Indicate that the UART is now ready to send */
    UART_Ready = 1;
}

/* UART Receive Complete Interrupt Function */
SIGNAL(SIG_UART_RECV)      
{
    /* Indicate that the UART has received a character */
    UART_ReceivedChar = 1;

    /* Store received character */
    UART_RxChar = inp(UDR);
	
	//Check our start delimiter an process next byte if he is matched
	if((UART_RxChar == 0xF1) || (UART_RxChar == 0xF0)){
		COMMAND = UART_RxChar;
		STATE = WAIT_SPEED;
	}
	else if (STATE == WAIT_VITESSE){
		VITESSE = UART_RxChar;
		STATE = WAIT_ANGLE;
	}
	else if (STATE == WAIT_ANGLE){
		ANGLE = UART_RxChar;
		STATE = WAIT_COMMAND;
		PACKET_READY = 1;

	}
	else{
		STATE = WAIT_COMMAND;
	}

	if(ECHO == 1)
	{
		UDR = UART_RxChar;	
	}
}

u08 GetCommandRemote()
{
	return  COMMAND;
}

float GetSpeedRemote()
{
	return (float)SPEED;
}

float GetAngleRemote()
{
	return (float)ANGLE;
}

void UART_DisableEcho(void)
{
	ECHO = 0;
}

void UART_EnableEcho(void)
{
	ECHO = 1;
}

void UART_SendByte(u08 Data)
{   
    /* wait for UART to become available */
    while(!UART_Ready);
    UART_Ready = 0;
    /* Send character */
    outp( Data, UDR );
}

u08 UART_ReceiveByte(void)
{
    /* wait for UART to indicate that a character has been received */
    while(!UART_ReceivedChar);
    UART_ReceivedChar = 0;
    /* read byte from UART data buffer */
    return UART_RxChar;
}

void UART_PrintfProgStr(PGM_P pBuf)
{
    /* wait for UART to become available */
    while(!UART_Ready);
    UART_Ready = 0;
    /* Indicate to ISR the string to be sent */
    pUART_Buffer = (u08*) pBuf;
    /* Send first character */
    outp( *pUART_Buffer, UDR );
}

void UART_PrintfEndOfLine(void)
{
    /* wait for UART to become available */
    while(!UART_Ready);
    UART_Ready = 0;
    /* Indicate to ISR the string to be sent */
    pUART_Buffer = (u08*) UART_pszEndOfLine;
    /* Send first character */
    outp( pgm_read_byte(pUART_Buffer), UDR );
}

void UART_PrintfU4(u08 Data)
{
    /* Send 4-bit hex value */
    u08 Character = Data&0x0f;
    if (Character>9)
    {
        Character+='A'-10;
    }
    else
    {
        Character+='0';
    }
    UART_SendByte(Character);
}

void UART_Printfu08(u08 Data)
{
    /* Send 8-bit hex value */
    UART_PrintfU4(Data>>4);
    UART_PrintfU4(Data   );
}

void UART_Printfu16(u16 Data)
{
    /* Send 16-bit hex value */
    UART_PrintfU4(Data>>12);
    UART_PrintfU4(Data>> 8);
    UART_PrintfU4(Data>> 4);
    UART_PrintfU4(Data    );
}

void UART_Init(void)
{
    UART_Ready        = 1;
    UART_ReceivedChar = 0;
    pUART_Buffer      = 0;
    /* configure asynchronous operation, no parity, 1 stop bit, 8 data bits, Tx on rising edge */
    outp((1<<URSEL)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|(0<<USBS)|(1<<UCSZ1)|(1<<UCSZ0)|(0<<UCPOL),UCSRC);       
    /* enable RxD/TxD and ints */
    outp((1<<RXCIE)|(1<<TXCIE)|(1<<RXEN)|(1<<TXEN)|(0<<UCSZ2),UCSRB);       
    /* set baud rate */
    outp((u08)(UART_BAUD_SELECT >> 8), UBRRH);          
    outp((u08)(UART_BAUD_SELECT & 0x00FF), UBRRL);
	
	
}
