/******************************************************************************
 *
 *
 *****************************************************************************/

#ifndef I2C_H
#define I2C_H

#include "../include/global_vars.h"

#include <avr/interrupt.h>

#define CIRCULAR_BUFFER_SIZE 32 //////MUST BE CHANGE VALUE TO 70 TO ALLOW COMPILATION

/** Prototype */
void TWIInit(void);

void putDataOutBuf(u08 data);
u08 getDataOutBuf(void);
void putDataInBuf(u08 * ptr);
u08 * getDataInBuf(void);

void twiWrite(u08 address, u08 registre, u08 data);
void twiRead(u08 address, u08 registre, u08 *ptr);

/** Les variables */
u08 CircularBufferOut[CIRCULAR_BUFFER_SIZE];
u08 * CircularBufferIn[CIRCULAR_BUFFER_SIZE];

u08 CircularBufferOutEnd;
u08 CircularBufferOutIndex;
u08 CircularBufferInEnd;
u08 CircularBufferInIndex;


#endif
