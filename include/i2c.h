/*
    Title:    i2c.h
    Authors:  Mikael Ferland, Joël Brisson
    Date:     11/2015
    Purpose:  Header for i2c.c
    Software: AVR-GCC to compile
    Hardware: ATMega32 on STK500 board
    Note:     
*/
#ifndef I2C_H
#define I2C_H

#include "../include/global_vars.h"

#include <avr/interrupt.h>

#define CIRCULAR_BUFFER_SIZE 32

/* Initialysation for the Two Wire Interface */
void TWIInit(void);

void putDataOutBuf(u08 data);
u08 getDataOutBuf(void);
void putDataInBuf(u08 * ptr);
u08 * getDataInBuf(void);

void twiWrite(u08 address, u08 registre, u08 data);
void twiRead(u08 address, u08 registre, u08 *ptr);

/* The variables */
u08 CircularBufferOut[CIRCULAR_BUFFER_SIZE];
u08 * CircularBufferIn[CIRCULAR_BUFFER_SIZE];

u08 CircularBufferOutEnd;
u08 CircularBufferOutIndex;
u08 CircularBufferInEnd;
u08 CircularBufferInIndex;


#endif
