/*
    Title:    main.c
    Authors:  Mikael Ferland, Joël Brisson
    Date:     11/2015
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
#include "../include/i2c.h"

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
#define SET_DIR_L1		PIND2
#define SET_DIR_L2		PIND3
#define SET_DIR_R1		PIND6
#define SET_DIR_R2		PIND7
#define GET_L_DIR_PIN	PINA2
#define GET_R_DIR_PIN	PINA3
#define SPEED_REMOTE_MAX 0xC8
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
volatile u08 L_DIR_SENSOR, R_DIR_SENSOR;
volatile u08 STAB;
volatile u08 LED_Status = 0x00, OLD_LED_Status = 0x00, LED_Counter = 0x00;

volatile s08 R_SIGN, L_SIGN;

volatile u16 V_L_MOTOR_SENSOR_RAW, V_R_MOTOR_SENSOR_RAW;

volatile int V_L_MOTOR_SENSOR_ACCUMULATOR, V_R_MOTOR_SENSOR_ACCUMULATOR;

volatile float V_L_MOTOR_SENSOR_MEAN, V_R_MOTOR_SENSOR_MEAN;
volatile u16 L_COUNTER_FOR_MEAN, R_COUNTER_FOR_MEAN;

volatile u08 Ping_Side = 0;
volatile u08 Ping_Sensor_MSB, Ping_Sensor_LSB;
volatile u16 R_Ping_Sense, L_Ping_Sense, Closest_Sense;
volatile u08 Time_To_Ping = 0;

float   SPEED_REMOTE, ANGLE_REMOTE;
float	SPEED_SETPOINT, ANGLE_SETPOINT;
float	V_L_MOTOR_SENSOR_ENG, V_R_MOTOR_SENSOR_ENG;
float 	DUTY_R, DUTY_L;

float	L_slope_P, L_slope_N, R_slope_P, R_slope_N;

int		L_VMIN_N,L_VMAX_N, L_VMIN_P, L_VMAX_P;
int		R_VMIN_N,R_VMAX_N, R_VMIN_P, R_VMAX_P;

u16   DUTY_L_REG, DUTY_R_REG;

u16	  TT;

/* Functions ------------------------------------------------- */

//Print information on the remote consol
void print(PGM_P string){
		UART_DisableEcho();
		UART_SendByte(START_DEBUG);
		PRINT(string);	
		UART_SendByte(STOP_DEBUG);
		UART_EnableEcho();
}

//Set left motor to neutral
void l_motorToNeural(){
	PORTD &= ~( (1 << SET_DIR_L1) | (1 << SET_DIR_L2) );
}

//Set right motor to neutral
void r_motorToNeural(){
	PORTD &= ~( (1 << SET_DIR_R1) | (1 << SET_DIR_R2) );
}

//Set all motor to neutral
void robotToNeutral(){
	PORTD &= ~( (1 << SET_DIR_L1) | (1 << SET_DIR_L2) | (1 << SET_DIR_R1) | (1 << SET_DIR_R2) );
}

//Set left motor to forward
void l_forward_motor(){
	PORTD |=  (1 << SET_DIR_L1);
	PORTD &= ~(1 << SET_DIR_L2);
}

//Set right motor to forward
void r_forward_motor(){
	PORTD |=  (1 << SET_DIR_R1);
	PORTD &= ~(1 << SET_DIR_R2);
}

//Set left motor to reverse
void l_reverse_motor(){
	PORTD &= ~(1 << SET_DIR_L1);
	PORTD |=  (1 << SET_DIR_L2);
}

//Set right motor to reverse
void r_reverse_motor(){
	PORTD &= ~(1 << SET_DIR_R1);
	PORTD |=  (1 << SET_DIR_R2);
}

//Set all motor to stop
void stopRobot(){
	OCR1A = 0;
	OCR1B = 0;
	PORTD |= ( (1 << SET_DIR_L1) | (1 << SET_DIR_L2) | (1 << SET_DIR_R1) | (1 << SET_DIR_R2) );
}

//Simple short delay of 1.5mS
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

//Wait for 11 ADC Sample on both motor side
void waitForSample(){
	while((L_COUNTER_FOR_MEAN < 12) && (R_COUNTER_FOR_MEAN < 12));
}

//Reset all value use for Mean
void resetMeanValue(){
	V_L_MOTOR_SENSOR_ACCUMULATOR =0;
	V_R_MOTOR_SENSOR_ACCUMULATOR =0;
	V_L_MOTOR_SENSOR_MEAN = 0.0;
	V_R_MOTOR_SENSOR_MEAN = 0.0;
	L_COUNTER_FOR_MEAN =0;
	R_COUNTER_FOR_MEAN =0;
}

//Calibration of max, neutral and min value for the right and left motor
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

	//Calculate mean value
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

	//Calculate mean value
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

	//Calculate mean value
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

	//Calculate mean value
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

//Update the LED state
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

//Speed command must be between -1 and 1
//Return 33 to 68 for speed_command > 0
u08 SetSonarRange(float speed_command){	
	return  (0 <= speed_command) ? (u08)(( (68.7274-33.8837) * speed_command) + 33.8837) : 0;
}

//Convert speed command into a gain in range of 8 to 12
//Speed command must be between -1 and 1
//Return 8 to 12 for speed_command > 0
u08 SetSonarGain(float speed_command){
	return  (0 <= speed_command) ? (u08)((4.0 * speed_command) + 8.0) : 0;
}

//Interrupt for: Timer 1 flag
SIGNAL(SIG_OVERFLOW1)
{
	TIME_TO_COMPUTE_PWM = 1;
	Time_To_Ping++;
}

//Interrupt for: End of ADC conversion
SIGNAL(SIG_ADC)
{
	if (STAB == 0){
		if(CURRENT_CHANNEL == 0){
			L_DIR_SENSOR = PINA & (1 << GET_L_DIR_PIN);
			L_SIGN = (L_DIR_SENSOR ==0) ? 1 : -1;

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
			R_DIR_SENSOR = PINA & (1 << GET_R_DIR_PIN);			
			R_SIGN = (R_DIR_SENSOR ==0) ? 1 : -1;

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

	resetMeanValue();
	
	TIME_TO_COMPUTE_PWM = 0;
	CURRENT_CHANNEL = 0;
	DUTY_R = 0.0;
	DUTY_L = 0.0;
	STAB = 30;
	TT=500;
	
	/* Initialisation */
	ADC_Init();
	timer_Init();
	
	//Enable Interrupt		
	sei();
	
	calibrate();

	TWIInit();
	UART_Init();

	//Print on Remote console Calibration values
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
    		//Once receive all information commands from Remote
		if(PACKET_READY == 1){
			wdt_reset();
			SPEED_REMOTE = GetSpeedRemote();
			ANGLE_REMOTE = GetAngleRemote();

			SPEED_SETPOINT = (0.01 * SPEED_REMOTE) -1.0;		
			ANGLE_SETPOINT = (2.0 * PI) * (ANGLE_REMOTE / 180.0);

			PACKET_READY = 0;
			
			//Update LED value
			LED_Status |= 0x02;	
		}
		
		//When its time to send a Sonar ping
		if(Time_To_Ping >= 10)
		{
			if(Ping_Side == 0)//Store value of Left Ping Sense, Read value of Right Ping Sensor, Ping on Left Sensor
			{
				L_Ping_Sense = (Ping_Sensor_MSB << 8) + Ping_Sensor_LSB; //Store Value of Ping Sense
				if(L_Ping_Sense != 0x0FFF)		// If obstacle detected, update coresponding LED
					LED_Status |= (1<<LObstacle_LED); 
				else
					LED_Status &= ~(1 << LObstacle_LED);

				twiWrite(0xE0, 0x02, SetSonarRange(SPEED_SETPOINT)); //Set Sonar Range depending on speed
				putDataOutBuf(0xFE);
				twiRead(0xE2, 0x02, &Ping_Sensor_MSB);		//Read data from the Ping Sensor
				putDataOutBuf(0xFE);
				twiRead(0xE2, 0x03, &Ping_Sensor_LSB);
				putDataOutBuf(0xFE);
				twiWrite(0xE0, 0x00, 0x51); 		//Send the Ping Command
				putDataOutBuf(0xFF);
				TWCR |= (1<<TWSTA); 			//start TWi transmition
				LED_Status |= (1 << LSonar_LED); //toggle the Ping LED

				Ping_Side = 1;
			}
			else // Store value of Right Ping Sense, Read value of Left Ping Sensor, Ping on right Sensor
			{
				R_Ping_Sense = (Ping_Sensor_MSB << 8) + Ping_Sensor_LSB; // Store Value of Pin Sense
				if(R_Ping_Sense != 0x0FFF)		// If obstacle detected, update coresponding LED
					LED_Status |= (1<<RObstacle_LED);
				else
					LED_Status &= ~(1 << RObstacle_LED);
				
				twiWrite(0xE2, 0x02, SetSonarRange(SPEED_SETPOINT)); //Set Sonar Range depending on speed
				putDataOutBuf(0xFE);
				twiRead(0xE0, 0x02, &Ping_Sensor_MSB);		//Read data from the Ping Sensor of the other side
				putDataOutBuf(0xFE);
				twiRead(0xE0, 0x03, &Ping_Sensor_LSB);
				putDataOutBuf(0xFE);
				twiWrite(0xE2, 0x00, 0x51);		//Send the Ping Command
				putDataOutBuf(0xFF);
				TWCR |= (1<<TWSTA);				//start TWi transmition
				LED_Status |= (1 << RSonar_LED); //toggle the Ping LED

				Ping_Side = 0;
			}

			//Check wich side Sense the closest obstacle
			if(R_Ping_Sense < L_Ping_Sense)
				Closest_Sense = R_Ping_Sense;
			else
				Closest_Sense = L_Ping_Sense;

			Time_To_Ping = 0; // reset the Time_To_Ping value
		}
		
		//at every 5ms
		if(TIME_TO_COMPUTE_PWM==1){			
			
			//Calculate mean
			V_L_MOTOR_SENSOR_MEAN = (float)(V_L_MOTOR_SENSOR_ACCUMULATOR / ((int)L_COUNTER_FOR_MEAN));
			V_R_MOTOR_SENSOR_MEAN = (float)(V_R_MOTOR_SENSOR_ACCUMULATOR / ((int)R_COUNTER_FOR_MEAN));	
			
			//Verify for direction
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

			if((Closest_Sense <=0x00FF) && (Closest_Sense >= 0x01F))
				CalculPWM((((float)Closest_Sense)/((float)0x00FF))*SPEED_SETPOINT, ANGLE_SETPOINT, V_L_MOTOR_SENSOR_ENG, V_R_MOTOR_SENSOR_ENG, &DUTY_L, &DUTY_R);
			else if(Closest_Sense < 0x01F)
				CalculPWM(0, ANGLE_SETPOINT, V_L_MOTOR_SENSOR_ENG, V_R_MOTOR_SENSOR_ENG, &DUTY_L, &DUTY_R);
			else
				CalculPWM(SPEED_SETPOINT, ANGLE_SETPOINT, V_L_MOTOR_SENSOR_ENG, V_R_MOTOR_SENSOR_ENG, &DUTY_L, &DUTY_R);
			
			//robotToNeutral();
			if((L_COUNTER_FOR_MEAN > 10) && (R_COUNTER_FOR_MEAN > 10)){
				if (0xF1 == GetCommandRemote()){
					if((DUTY_L > 0.0)){
						DUTY_L_REG = (u16)(9999.0 * DUTY_L);
					}
					else if((DUTY_L < 0.0)){
						DUTY_L_REG = (u16)(-1.0 * (9999.0 * DUTY_L));
					}
					else{
						DUTY_L_REG = 0.0;
					}


					if((DUTY_R > 0.0)){
						DUTY_R_REG = (u16)(9999.0 * DUTY_R);
					}
					else if((DUTY_R < 0.0)){
						DUTY_R_REG = (u16)(-1.0 * (9999.0 * DUTY_R));
					}
					else{
						DUTY_R_REG = 0.0;
					}

					if (DUTY_L == 0.0){
						l_motorToNeural();
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

					resetMeanValue();
				}
				else{
					stopRobot();
					OCR1B = 0;
					OCR1A = 0;
				}
			}

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
