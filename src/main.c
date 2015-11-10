/*
    Title:    main.c
    Authors:  Mikael Ferland, Joël Brisson
    Date:     09/2015
    Purpose:  Control a robot through UART and SPI
    needed
    Software: AVR-GCC to compile
    needed
    Hardware: ATMega32 on STK500 board
    Note:     
*/

/* Custom librairies ----------------------------------------- */
#include "../include/global_vars.h"
#include "../include/uart.h"
#include "../include/timer.h"
#include "../include/moteur.h"
#include "../include/adc.h"
#include "../include/i2c_caneva.h"

/* General librairies ---------------------------------------- */
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdint.h>
#include <stdio.h>


/* Constants ------------------------------------------------ */
#define PI      (3.1415926535897932384626433832795)
#define START_DEBUG 	0xFE
#define STOP_DEBUG		0xFF
#define SET_DIR_G1		PIND2
#define SET_DIR_G2		PIND3
#define SET_DIR_D1		PIND6
#define SET_DIR_D2		PIND7
#define GET_G_DIR_PIN	PINA2
#define GET_D_DIR_PIN	PINA3
#define VITESSE_TELEGUIDAGE_MAX 0xC8
#define CALIB_PIN		PINA4

/* LED-------------------------------------------------------- */
#define Command_LED 	1
#define RObstacle_LED 	2
#define RSonar_LED 		3
#define LObstacle_LED	4
#define LSonar_LED 		5
#define WAIT_LED 		6
#define RUN_LED 		7


/* Macro------------------------------------------------------ */
#define PRINT_DEBUG(string) print((PGM_P) string)

/*Global Variables ------------------------------------------- */
volatile u08 TIME_TO_COMPUTE_PWM;
volatile u08 CURRENT_CHANNEL;
volatile u08 G_DIR_SENSOR, D_DIR_SENSOR;
volatile u08 STAB;
volatile u08 LED_Status = 0x00, OLD_LED_Status = 0x00, LED_Counter = 0x00;

volatile s08 R_SIGN, L_SIGN;

volatile u16 V_L_MOTOR_SENSOR_RAW, V_R_MOTOR_SENSOR_RAW;

volatile int V_L_MOTOR_SENSOR_ACCUMULATOR, V_R_MOTOR_SENSOR_ACCUMULATOR;

volatile float V_L_MOTOR_SENSOR_MEAN, V_R_MOTOR_SENSOR_MEAN;
volatile u16 L_COUNTER_FOR_MEAN, R_COUNTER_FOR_MEAN;


float   VITESSE_TELEGUIDAGE, ANGLE_TELEGUIDAGE;
float	VITESSE_CONSIGNE, ANGLE_CONSIGNE;
float	V_L_MOTOR_SENSOR_ENG, V_R_MOTOR_SENSOR_ENG;
float 	DUTY_R, DUTY_L;

float	L_slope_P, L_slope_N, R_slope_P, R_slope_N;

int		L_VMIN_N,L_VMAX_N, L_VMIN_P, L_VMAX_P;
int		R_VMIN_N,R_VMAX_N, R_VMIN_P, R_VMAX_P;

u16   DUTY_L_REG, DUTY_R_REG;

u16	  TT;

/* Functions ------------------------------------------------- */
void print(PGM_P string){
		UART_DisableEcho();
		UART_SendByte(START_DEBUG);
		PRINT(string);	
		UART_SendByte(STOP_DEBUG);
		UART_EnableEcho();
}

void l_motorToNeural(){
	PORTD &= ~( (1 << SET_DIR_G1) | (1 << SET_DIR_G2) );
}
void r_motorToNeural(){
	PORTD &= ~( (1 << SET_DIR_D1) | (1 << SET_DIR_D2) );
}
void robotToNeutral(){
	PORTD &= ~( (1 << SET_DIR_G1) | (1 << SET_DIR_G2) | (1 << SET_DIR_D1) | (1 << SET_DIR_D2) );
}
void l_forward_motor(){
	PORTD |=  (1 << SET_DIR_G1);
	PORTD &= ~(1 << SET_DIR_G2);
}
void r_forward_motor(){
	PORTD |=  (1 << SET_DIR_D1);
	PORTD &= ~(1 << SET_DIR_D2);
}
void l_reverse_motor(){
	PORTD &= ~(1 << SET_DIR_G1);
	PORTD |=  (1 << SET_DIR_G2);
}
void r_reverse_motor(){
	PORTD &= ~(1 << SET_DIR_D1);
	PORTD |=  (1 << SET_DIR_D2);
}
void stopRobot(){
	OCR1A = 0;
	OCR1B = 0;
	PORTD |= ( (1 << SET_DIR_G1) | (1 << SET_DIR_G2) | (1 << SET_DIR_D1) | (1 << SET_DIR_D2) );
}
void delay(){
	u16 count = 1000;
	u16 count1 = 25;
	while(count != 0){
		count--;

		while(count1 != 0){
			count1--;

		}
		count1 = 25;

	}
}

void waitForSample(){
	while((L_COUNTER_FOR_MEAN < 12) && (R_COUNTER_FOR_MEAN < 12));
}

void resetMeanValue(){
	V_L_MOTOR_SENSOR_ACCUMULATOR =0;
	V_R_MOTOR_SENSOR_ACCUMULATOR =0;
	V_L_MOTOR_SENSOR_MEAN = 0.0;
	V_R_MOTOR_SENSOR_MEAN = 0.0;
	L_COUNTER_FOR_MEAN =0;
	R_COUNTER_FOR_MEAN =0;
}

void calibrate(){

	//Vmax+ motors
	stopRobot();
	delay();
	
	PORTA |=  (1 << CALIB_PIN);
	l_forward_motor();
	r_forward_motor();
	delay();

	resetMeanValue();
	waitForSample();

	V_L_MOTOR_SENSOR_MEAN = (float)(V_L_MOTOR_SENSOR_ACCUMULATOR / ((int)L_COUNTER_FOR_MEAN));
	V_R_MOTOR_SENSOR_MEAN = (float)(V_R_MOTOR_SENSOR_ACCUMULATOR / ((int)R_COUNTER_FOR_MEAN));
	
	L_VMAX_P = V_L_MOTOR_SENSOR_MEAN;
	R_VMAX_P = V_R_MOTOR_SENSOR_MEAN;
	PORTA &= ~(1 << CALIB_PIN);

	

	//Vzero+ motors
	stopRobot();
	delay();

	resetMeanValue();
	waitForSample();

	V_L_MOTOR_SENSOR_MEAN = (float)(V_L_MOTOR_SENSOR_ACCUMULATOR / ((int)L_COUNTER_FOR_MEAN));
	V_R_MOTOR_SENSOR_MEAN = (float)(V_R_MOTOR_SENSOR_ACCUMULATOR / ((int)R_COUNTER_FOR_MEAN));

	L_VMIN_P = V_L_MOTOR_SENSOR_MEAN;
	R_VMIN_P = V_R_MOTOR_SENSOR_MEAN;
	
	//Vmax- motors
	stopRobot();
	delay();

	PORTA |=  (1 << CALIB_PIN);
	l_reverse_motor();
	r_reverse_motor();
	delay();

	resetMeanValue();
	waitForSample();

	V_L_MOTOR_SENSOR_MEAN = (float)(V_L_MOTOR_SENSOR_ACCUMULATOR / ((int)L_COUNTER_FOR_MEAN));
	V_R_MOTOR_SENSOR_MEAN = (float)(V_R_MOTOR_SENSOR_ACCUMULATOR / ((int)R_COUNTER_FOR_MEAN));

	L_VMAX_N = V_L_MOTOR_SENSOR_MEAN;
	R_VMAX_N = V_R_MOTOR_SENSOR_MEAN;
	PORTA &= ~(1 << CALIB_PIN);

	//Vzero- motors
	stopRobot();
	delay();
	
	resetMeanValue();
	waitForSample();

	V_L_MOTOR_SENSOR_MEAN = (float)(V_L_MOTOR_SENSOR_ACCUMULATOR / ((int)L_COUNTER_FOR_MEAN));
	V_R_MOTOR_SENSOR_MEAN = (float)(V_R_MOTOR_SENSOR_ACCUMULATOR / ((int)R_COUNTER_FOR_MEAN));
	
	L_VMIN_N = V_L_MOTOR_SENSOR_MEAN;
	R_VMIN_N = V_R_MOTOR_SENSOR_MEAN;

	L_slope_P = (1.0 / (L_VMAX_P - L_VMIN_P));
	R_slope_P = (1.0 / (R_VMAX_P - R_VMIN_P));
	L_slope_N = (1.0 / (L_VMAX_N - L_VMIN_N));
	R_slope_N = (1.0 / (R_VMAX_N - R_VMIN_N));

	resetMeanValue();
}

void UpdateLED()
{
	// when a flashing led has his status bit set to '1',
	// toggle the LED output then put his status to '0'
	if((LED_Status & 0x02))
	{
		//toggle PIN State
		PORTB = (PORTB ^ 0x02);
	}

	if((LED_Status & 0x08))
	{
		//toggle PIN State
		PORTB = (PORTB ^ 0x08);
	}

	if((LED_Status & 0x20))
	{
		//toggle PIN State
		PORTB = (PORTB ^ 0x20);
	}


	// Mask all LED that does not flash
	// then write them to the LED PORT
	PORTB = (0xD4 | PORTB) & ~(0xD4 & LED_Status);

	LED_Status = LED_Status & 0xD4;
	OLD_LED_Status = LED_Status;
}

//Timer 1 flag
SIGNAL(SIG_OVERFLOW1)
{
	TIME_TO_COMPUTE_PWM = 1;
}

SIGNAL(SIG_ADC)
{
	if (STAB == 0){
		if(CURRENT_CHANNEL == 0){
			G_DIR_SENSOR = PINA & (1 << GET_G_DIR_PIN);
			L_SIGN = (G_DIR_SENSOR ==0) ? 1 : -1;

			V_L_MOTOR_SENSOR_RAW =  ADC & (0x3FF); //10 bits mask

			if (L_SIGN > 0){
				V_L_MOTOR_SENSOR_ACCUMULATOR += ((int)V_L_MOTOR_SENSOR_RAW);
			}
			else{
				V_L_MOTOR_SENSOR_ACCUMULATOR -= ((int)V_L_MOTOR_SENSOR_RAW);
			}
			
			L_COUNTER_FOR_MEAN++;
		}
		else{
			D_DIR_SENSOR = PINA & (1 << GET_D_DIR_PIN);			
			R_SIGN = (D_DIR_SENSOR ==0) ? 1 : -1;

			V_R_MOTOR_SENSOR_RAW =  ADC & (0x3FF); //10 bits mask

			if(R_SIGN >0){
				V_R_MOTOR_SENSOR_ACCUMULATOR += ((int)V_R_MOTOR_SENSOR_RAW);
			}
			else{
				V_R_MOTOR_SENSOR_ACCUMULATOR -= ((int)V_R_MOTOR_SENSOR_RAW);
			}
			
			R_COUNTER_FOR_MEAN++;

		}
		
		CURRENT_CHANNEL = !(CURRENT_CHANNEL & 0x01);
		ADMUX = CURRENT_CHANNEL;
				
		STAB = 3;
		
	}
	else{
		STAB--;
	}

}

int main(void)
{
	//Calibration output at PINA4
	DDRA = 0x10;

	//**DDRD: 1 = OUTPUT**
    //Pin 7 to 2 on Port D are output
	DDRD = 0xFC;

	//Pin 7 to 1 on Port B are Output. *LED*
	DDRB = 0xFE; //NC on port 0 pin 0, keep it input
	
	PORTB=0xFF;

	V_L_MOTOR_SENSOR_ACCUMULATOR =0;
	V_R_MOTOR_SENSOR_ACCUMULATOR =0;
	V_L_MOTOR_SENSOR_MEAN = 0.0;
	V_R_MOTOR_SENSOR_MEAN = 0.0;
	L_COUNTER_FOR_MEAN = 0;
	R_COUNTER_FOR_MEAN = 0;

	
	TIME_TO_COMPUTE_PWM = 0;
	CURRENT_CHANNEL = 0;
	DUTY_R = 0.0;
	DUTY_L = 0.0;
	STAB = 30;
	TT=500;
	
	/* Initialisation */
	ADC_Init();
	timer_Init();
			
	sei();
	
	calibrate();

	//TWIInit();
	UART_Init();

	UART_DisableEcho();	
	UART_SendByte(START_DEBUG);
	UART_PrintfProgStr("Calibration done! ");
	UART_PrintfEndOfLine();
	UART_PrintfProgStr("Vmax pos ->");
	UART_Printfu16(L_VMAX_P);
	UART_PrintfEndOfLine();
	UART_PrintfProgStr("Vmin pos ->");
	UART_Printfu16(L_VMIN_P);
	UART_PrintfEndOfLine();
	UART_PrintfProgStr("Vmax neg ->");
	UART_Printfu16(L_VMAX_N);
	UART_PrintfEndOfLine();
	UART_PrintfProgStr("Vmax neg ->");
	UART_Printfu16(L_VMIN_N);
	UART_PrintfEndOfLine();
	UART_SendByte(STOP_DEBUG);
	UART_EnableEcho();

	l_forward_motor();
	r_forward_motor();

	/* Wait for Switch to be press before robot start moving*/
	while(PINA & 0x40) // Wait for SW6 to be press
	{
		//PORTB &= ~(1<<WAIT_LED); // set on WAIT_LED
		LED_Status |= (1<<WAIT_LED);
		UpdateLED();
	}
	LED_Status |= (1<<RUN_LED);
	LED_Status &= ~(1<<WAIT_LED);
	
    for (;;) {  /* loop forever */
		if(PACKET_READY == 1){
			wdt_reset();

			VITESSE_TELEGUIDAGE = GetVitesseTeleguidage();
			ANGLE_TELEGUIDAGE = GetAngleTeleguidage();

			VITESSE_CONSIGNE = (0.01 * VITESSE_TELEGUIDAGE) -1.0;		
			ANGLE_CONSIGNE	 = (2.0 * PI) * (ANGLE_TELEGUIDAGE / 180.0);
			
			/*//Debug*********
			UART_DisableEcho();
			UART_SendByte(START_DEBUG);
			UART_PrintfProgStr("Vitesse: ");
			UART_Printfu08((u08) VITESSE_TELEGUIDAGE);
			UART_PrintfProgStr(" ");	
			UART_PrintfProgStr(" Angle: ");
			UART_Printfu08((u08) ANGLE_TELEGUIDAGE);
			UART_PrintfProgStr(" ");					
			UART_SendByte(STOP_DEBUG);
			UART_EnableEcho();
			*/	
			
			PACKET_READY = 0;
			//Update LED value
			LED_Status |= 0x02;	
		}
		
		if(TIME_TO_COMPUTE_PWM==1){			
			
			V_L_MOTOR_SENSOR_MEAN = (float)(V_L_MOTOR_SENSOR_ACCUMULATOR / ((int)L_COUNTER_FOR_MEAN));
			V_R_MOTOR_SENSOR_MEAN = (float)(V_R_MOTOR_SENSOR_ACCUMULATOR / ((int)R_COUNTER_FOR_MEAN));	
			
			if(V_L_MOTOR_SENSOR_MEAN > 0.0){
				V_L_MOTOR_SENSOR_ENG = (V_L_MOTOR_SENSOR_MEAN - L_VMIN_P) * L_slope_P;	
			}
			else{
				V_L_MOTOR_SENSOR_ENG = (V_L_MOTOR_SENSOR_MEAN - L_VMIN_N) * (-L_slope_N);
			}

			if(V_R_MOTOR_SENSOR_MEAN > 0.0){
				V_R_MOTOR_SENSOR_ENG = (V_R_MOTOR_SENSOR_MEAN - R_VMIN_P) * R_slope_P;
			}
			else{	
				V_R_MOTOR_SENSOR_ENG = (V_R_MOTOR_SENSOR_MEAN - R_VMIN_N ) * (-R_slope_N);
			}

			CalculPWM(VITESSE_CONSIGNE, ANGLE_CONSIGNE, V_L_MOTOR_SENSOR_ENG, V_R_MOTOR_SENSOR_ENG, &DUTY_L, &DUTY_R);
			
			robotToNeutral();

			if (0xF1 == GetCommandeTeleguidage()){
				if((DUTY_L > 0.0)){
					DUTY_L_REG = (u16)(9999.0 * DUTY_L);
				}
				else{
					DUTY_L_REG = (u16)(-1.0 * (9999.0 * DUTY_L));
				}
				if((DUTY_R > 0.0)){
					DUTY_R_REG = (u16)(9999.0 * DUTY_R);
				}
				else{
					DUTY_R_REG = (u16)(-1.0 * (9999.0 * DUTY_R));
				}

				if (DUTY_L == 0.0){
					l_motorToNeural();
					OCR1B = 0;
				}
				else{
					if(DUTY_L > 0.0){
						l_forward_motor();
					}
					else if(DUTY_L < 0.0){
						l_reverse_motor();
					}


				}

				if (DUTY_R == 0.0){
					r_motorToNeural();
					OCR1A = 0;
				}
				else{
					if(DUTY_R > 0.0){
						r_forward_motor();
					}
					else if(DUTY_R < 0.0){
						r_reverse_motor();
					}

					
				}

				OCR1B = DUTY_L_REG;
				OCR1A = DUTY_R_REG;

				/*//Debug*********
				UART_DisableEcho();
				UART_SendByte(START_DEBUG);
				UART_PrintfProgStr(" Counter: ");
				UART_Printfu16(L_COUNTER_FOR_MEAN);
				UART_SendByte(STOP_DEBUG);
				UART_EnableEcho();
				*/

				resetMeanValue();
						
			}

			else{
				stopRobot();
				OCR1B = 0;
				OCR1A = 0;
			}


			/*UART_DisableEcho();
			UART_SendByte(START_DEBUG);
			UART_Printfu08(G_DIR_SENSOR);
			UART_PrintfProgStr(" ");
			UART_Printfu16(V_L_MOTOR_SENSOR_RAW);
			UART_PrintfProgStr(" ");
			UART_Printfu16(V_L_MOTOR_SENSOR_MEAN);
			UART_PrintfProgStr(" ");
			UART_Printfu08(D_DIR_SENSOR);
			UART_PrintfProgStr(" ");
			UART_Printfu16(V_R_MOTOR_SENSOR_RAW);
			UART_PrintfProgStr(" ");
			UART_Printfu16(V_R_MOTOR_SENSOR_MEAN);
			UART_PrintfProgStr("DUTY_L: ");
			UART_Printfu16(DUTY_L_REG);
			UART_PrintfProgStr(" DUTY_R: ");
			UART_Printfu16(DUTY_R_REG);*/
			
			TIME_TO_COMPUTE_PWM = 0;			

		}

		//if LED value has change, update LED PORT
		if(OLD_LED_Status != LED_Status)
		{
			UpdateLED();
		}

		// if SW7 is press, stop all motor and wait for SW6
		if(!(PINA & 0x80)){
			//stop all motor
			stopRobot();
			/*//PORTB = (PORTB | 0x40) & 0x7F; // turn on LED 6 and LED 7 off
			PORTB &= ~(1 << 6);
			PORTB |= (1 << 7);
			*/
		
			LED_Status &= ~(1 << RUN_LED);
			while(PINA & 0x40)
				{	
					LED_Status |= (1<<WAIT_LED);
					UpdateLED();
				}
			
			//PORTB = PORTB & 0xBF; // turn off LED 6
			//PORTB |= (1 << 6);
			LED_Status |= (1<<RUN_LED);	
			LED_Status &= ~(1 << WAIT_LED);
		}
				
    }
}
