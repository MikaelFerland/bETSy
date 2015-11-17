/*
    Title:    timer.h
    Authors:  Mikael Ferland, Joël Brisson
    Date:     07/10/2015
    Purpose:  Header for timer.c
    Software: AVR-GCC to compile
    Hardware: ATMega32 on STK500 board
    Note:     
*/

#ifndef __TIMER_H__
#define __TIMER_H__

#include <avr/io.h>

#define outp(val,port) port = val
#define inp(port) port

/* Global functions */
extern void timer_Init           (void);


#endif
