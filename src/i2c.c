/*
    Title:    i2c.c
    Authors:  Mikael Ferland, Joël Brisson
    Date:     12/11/2015
    Purpose:  Initialysation, writing and reading for TWI between master and slaves, using i2c protocol.
    Software: AVR-GCC to compile
    Hardware: ATMega32 on STK500 board
    Note:     0xFE and 0xFF is reserved by the system for restart and stop condition.
*/

#include "../include/i2c.h"

u08 buffer;		//Store value temporary to execute decision base on value
u08 *buffer_ptr;	//Address of the "buffer" variable

// Two Wire Interface initialisation and write starting parameter to Ping Sensor
void TWIInit()
{
	/*Set TWPS and TWS to 0 in the TWSR register
		TWPS is the Prescaler of TWI
		TWBR is the value of the TWI Bit Rate Register
		
		Use the following formula to calculate an SCL frequency of 10kHz
		SCL frequency = (CPU Clock)/(16+(TWBR)*4^(TWPS))
	*/
	
	//TWSR = (bits 7..3) TWS and (bits 1..0) TWPS
	//Set prescaler to 1, clear all TWI Status
	TWSR = 0;
	
	//TWBR = TWI Bit Rate
	//Set bit rate to 0xC6
	TWBR = 0xC6;

	//TWCR = TWI Control Register
	//Clear Interrupt Flag, Enable TWI Interrupt, Enable TWI Acknowledge Bit, Enable TWI
	TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
	
	//Set some initial value to both ping sensor
	twiWrite(0xE0, 0x02, 0x5A);
	putDataOutBuf(0xFE);	//Send restart
	twiWrite(0xE2, 0x02, 0x5A);
	putDataOutBuf(0xFE);
	twiWrite(0xE0, 0x01, 0x0A);
	putDataOutBuf(0xFE);
	twiWrite(0xE0, 0x01, 0x0A);
	putDataOutBuf(0xFF);	//Send stop
	
	TWCR |= (1<<TWSTA); 	// then start the TWI
}


/******************************************************************************
 * Insert in buffer out
 *****************************************************************************/
void putDataOutBuf(u08 data){

	CircularBufferOutEnd++;
	CircularBufferOutEnd %= CIRCULAR_BUFFER_SIZE;
	CircularBufferOut[CircularBufferOutEnd] = data;

}


/******************************************************************************
 * Retrieve from buffer out
 *****************************************************************************/
u08 getDataOutBuf(void){

	CircularBufferOutIndex++;
	CircularBufferOutIndex %= CIRCULAR_BUFFER_SIZE;
	return (u08)CircularBufferOut[CircularBufferOutIndex];

}


/******************************************************************************
 * Insert in buffer in
 *****************************************************************************/
void putDataInBuf(u08 * ptr){

	CircularBufferInEnd++;
	CircularBufferInEnd %= CIRCULAR_BUFFER_SIZE;
	CircularBufferIn[CircularBufferInEnd] = ptr;

}


/******************************************************************************
 * Retrieve from buffer in
 *****************************************************************************/
u08 * getDataInBuf(void){

	CircularBufferInIndex++;
	CircularBufferInIndex %= CIRCULAR_BUFFER_SIZE;
	return CircularBufferIn[CircularBufferInIndex];	

}


/******************************************************************************
 * Write on TWI bus
 *****************************************************************************/
void twiWrite(u08 address, u08 registre, u08 data){
		
	cli();
	putDataOutBuf(address);
	putDataOutBuf(registre);
	putDataOutBuf(data);
	sei();

}

/******************************************************************************
 * Read on TWI bus
 *****************************************************************************/
void twiRead(u08 address, u08 registre, u08 *ptr){

	cli();
	putDataOutBuf(address);
	putDataOutBuf(registre);
	putDataOutBuf(0xFE);
	putDataOutBuf(address+1);
	putDataInBuf(ptr);
	sei();

}


/******************************************************************************
 *Attach Interrupt signal and execute this routine
 *****************************************************************************/
SIGNAL(SIG_2WIRE_SERIAL) {
	
	u08 status  = TWSR & 0xF8;
		
	switch (status) {
		/*Case: start or restart condition */
		case	0x08: 	/* Start Condition */
		case	0x10: 	/* Restart Condition */
	
			TWDR = getDataOutBuf();
			//Reset TWI flag, Enable TWI, Enable TWI Acknoledge, Enable TWI Interrupt
			TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
			TWCR |= (1<<TWINT);
			
			break;
		
		/*case: write data on bus*/
		case	0x18: 	/* Address Write Ack */
		case	0x28: 	/* Data Write Ack */
		case	0x30: 	/* Date Write NoAck */
			
			buffer = getDataOutBuf();
			//Reset TWI flag, Enable TWI, Enable TWI Acknoledge, Enable TWI Interrupt
			TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
			
			// if start or restart condition is read on the bus
			if(buffer = 0xFF) 	// 0xFF will stop TWI
			{
				TWCR |= (1<<TWSTO);
			}
			else if(buffer == 0xFE)	// 0xFE will restart TWI 
			{
				TWCR |= (1<<TWSTA);
			}
			else			// Put the value from "buffer" on the register "TWDR"
			{
				TWDR = buffer;
				TWCR |= (1<<TWINT);
			}
			break;

		/*case: Read data on the bus*/
		case	0x50: 	/* Data Read Ack */
		case	0x58: 	/* Data Read NoAck */

			
			buffer = getDataInBuf();	//get Data from the circular buffer

			*buffer = TWDR;			//Write the value found in "TWDR" at the address of "buffer"
			
			buffer = getDataOutBuf();	//Read next value in the circular buffer
			//Reset TWI flag, Enable TWI, Enable TWI Acknoledge, Enable TWI Interrupt
			TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
			if(buffer == 0xFF)		// 0xFF will stop TWI
			{
				TWCR |= (1<<TWSTO);
			}
			else if(buffer == 0xFE)		// 0xFE will restart TWI
			{
				TWCR |= (1<<TWSTA);
			}
			else				// Put the value from "buffer" on the register "TWDR"
			{
				TWDR = buffer;
				TWCR |= (1<<TWINT);
			}
			break; 				// Added this break to hendel stop condition
		
		/*case: Received the Acknoledge from slave to read desired value*/
		case	0x40: /* Address Read Ack */

			//Reset TWI flag, Enable TWI, Enable TWI Acknoledge, Enable TWI Interrupt
			TWCR = (1<<TWEN) | (1<<TWIE);
			TWCR |= (1<<TWINT);
	
			break;
		
		/*case: No response from Sensor*/
		case	0x48: /* Address Read NoAck */
		case	0x20: /* Address Write NoAck */

			//Reset TWI flag, Enable TWI, Enable TWI Acknoledge, Enable TWI Interrupt
			TWCR =	(1<<TWEN) | (1<<TWSTO); //stop TWI
			TWCR |= (1<<TWINT);

			break;

		default : 
			/*
				Should not be use
			*/
			break;
	}
}
