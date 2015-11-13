/******************************************************************************
 *
 *
 *****************************************************************************/


#include "../include/i2c_caneva.h"

u08 buffer;
u08 *buffer_ptr;

// Two Wire Interface
void TWIInit()
{
	TWSR = 0;
	TWBR = 0xC6;

	//Enable Interrupt
	TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
	
	//Set some initial value to both ping sensor
	twiWrite(0xE0, 0x02, 0x5A);
	putDataOutBuf(0xFE);//Send restart
	twiWrite(0xE2, 0x02, 0x5A);
	putDataOutBuf(0xFE);
	twiWrite(0xE0, 0x01, 0x0A);
	putDataOutBuf(0xFE);
	twiWrite(0xE0, 0x01, 0x0A);
	putDataOutBuf(0xFF);//Send stop
	
	TWCR |= (1<<TWSTA); // then start the TWI
}


/******************************************************************************
 * Insérer dans le buffer out
 *****************************************************************************/
void putDataOutBuf(u08 data){

	CircularBufferOutEnd++;
	CircularBufferOutEnd %= CIRCULAR_BUFFER_SIZE;
	CircularBufferOut[CircularBufferOutEnd] = data;

}


/******************************************************************************
 * Retirer du buffer out
 *****************************************************************************/
u08 getDataOutBuf(void){

	CircularBufferOutIndex++;
	CircularBufferOutIndex %= CIRCULAR_BUFFER_SIZE;
	return (u08)CircularBufferOut[CircularBufferOutIndex];

}


/******************************************************************************
 * Insérer dans le buffer in
 *****************************************************************************/
void putDataInBuf(u08 * ptr){

	CircularBufferInEnd++;
	CircularBufferInEnd %= CIRCULAR_BUFFER_SIZE;
	CircularBufferIn[CircularBufferInEnd] = ptr;

}


/******************************************************************************
 * Retirer du buffer in
 *****************************************************************************/
u08 * getDataInBuf(void){

	CircularBufferInIndex++;
	CircularBufferInIndex %= CIRCULAR_BUFFER_SIZE;
	return CircularBufferIn[CircularBufferInIndex];	

}


/******************************************************************************
 * Écrire sur le bus twi
 *****************************************************************************/
void twiWrite(u08 address, u08 registre, u08 data){
		
	cli();
	putDataOutBuf(address);
	putDataOutBuf(registre);
	putDataOutBuf(data);
	sei();

}

/******************************************************************************
 * lire sur le bus
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
 *
 *****************************************************************************/
SIGNAL(SIG_2WIRE_SERIAL) {
	
	u08 status  = TWSR & 0xF8;
		
	switch (status) {
		case	0x08: /* Start Condition */
		case	0x10: /* Restart Condition */
			
			/* 
				Si  nous avons un start ou un restart condition alors il faut envoyer l'addr 
				qui est dans le buffer Out et Activer le bus sans start/stop 
			*/
			TWDR = getDataOutBuf();
			TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
			TWCR |= (1<<TWINT);
			
			break;

		case	0x18: /* Address Write Ack */
		case	0x28: /* Data Write Ack */
		case	0x30: /* Date Write NoAck */
			
			/* 
				Si  nous avons un data ou une addr d'écrit sur le bus, ensuite il peut y avoir un autre data, 
				un stop ou un restart. Il faut donc lire le buffer pour savoir quoi faire et configure 
				le bus en conséquence 
			*/
			buffer = getDataOutBuf();
			TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
			if(buffer = 0xFF)
			{
				TWCR |= (1<<TWSTO);
			}
			else if(buffer == 0xFE)
			{
				TWCR |= (1<<TWSTA);
			}
			else
			{
				TWDR = buffer;
				TWCR |= (1<<TWINT);
			}
			break;

		case	0x50: /* Data Read Ack */
		case	0x58: /* Data Read NoAck */

			/* 
				Une lecture à été effectué sur le bus, il faut donc la récupérer 
			*/
			buffer = getDataInBuf();

			*buffer = TWDR;
			
			buffer = getDataOutBuf();
			TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
			if(buffer == 0xFF)
			{
				TWCR |= (1<<TWSTO);
			}
			else if(buffer == 0xFE)
			{
				TWCR |= (1<<TWSTA);
			}
			else
			{
				TWDR = buffer;
				TWCR |= (1<<TWINT);
			}
			break; // Added this break to hendel stop condition

		case	0x40: /* Address Read Ack */

			/* 
				Puisqu'il n'y a pas de break dans les deux case 0x50 et 0x58, quand nous sommes ici
				nous avons soit lue la donnée ou envoyé l'addr à lire, il peut donc y avoir un stop, un
				start ou encore il faut placer le bus en mode lecture 
			*/
			TWCR = (1<<TWEN) | (1<<TWIE);
			TWCR |= (1<<TWINT);
	
			break;

		case	0x48: /* Address Read NoAck */
		case	0x20: /* Address Write NoAck */

			/* 
				Ici l'un des deux sonars n'a pas répondu, il faut donc tout stoper ou faire un restart
			    pour la prochaine trame qui peut être dans le buffer 
			*/
			TWCR =	(1<<TWEN) | (1<<TWSTO);
			TWCR |= (1<<TWINT);

			break;

		default : 
			/*
				Cette partie de code ne devrait pas être utile :)
			*/
			break;
	}
}
