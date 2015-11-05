#ifndef __UART_H__
#define __UART_H__

#include "../include/global_vars.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define outp(val,port) port = val
#define inp(port) port

/* UART Baud rate calculation */
#define UART_CPU               16000000UL   /* Crystal 16.000 Mhz */
#define UART_BAUD_RATE         9600         /* baud rate*/
#define UART_BAUD_SELECT       (UART_CPU/(UART_BAUD_RATE*16l)-1)

#define ATTENTE_COMMANDE	   0
#define ATTENTE_VITESSE		   1
#define ATTENTE_ANGLE		   2

/* Global functions */
extern void  UART_SendByte         (u08 Data);
extern u08   UART_ReceiveByte      (void);
extern void  UART_PrintfProgStr    (PGM_P pBuf);
extern void  UART_PrintfEndOfLine  (void);
extern void  UART_Printfu08        (u08 Data);
extern void  UART_Printfu16        (u16 Data);
extern void  UART_Init             (void);

extern void  UART_DisableEcho	   (void);
extern void  UART_EnableEcho	   (void);
extern u08	 GetCommandeTeleguidage(void);
extern float GetVitesseTeleguidage (void);
extern float GetAngleTeleguidage   (void);


/* Shared variable */
extern volatile u08  ECHO;
extern volatile u08  PACKET_READY;

/* Macros */
#define PRINT(string) (UART_PrintfProgStr((PGM_P) string))
#define EOL           UART_PrintfEndOfLine
#endif

