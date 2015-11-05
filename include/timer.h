#ifndef __TIMER_H__
#define __TIMER_H__

#include <avr/io.h>

#define outp(val,port) port = val
#define inp(port) port

/* Global functions */
extern void timer_Init           (void);

/* Macros */


#endif
