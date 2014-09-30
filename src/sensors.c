
/***********************************************
 *  
 *  ANTZ sensors
 *
 ***********************************************/
#include "antz.h"
#include <stdio.h>
#include "libpic30.h"
#include "sensors.h"
#include "adc.h"
#include "uart.h"
#include "spi.h"
#include "motors.h"
#include "systemtimer.h"

volatile int sensorsEnabled,cubeSensorsEnabled;
// raw sensor readings
volatile unsigned int leftSensor;
volatile unsigned int frontSensor;
volatile unsigned int rightSensor;
volatile unsigned int lineSensor[4];
volatile int headingError;
volatile int offsetError;

volatile unsigned int batteryVoltage;

// booleans holding presence/absence of walls
volatile unsigned char leftCube;
volatile unsigned char rightCube;
volatile unsigned char frontCube;
volatile unsigned char intersection=FALSE;
volatile unsigned char oldIntersection=FALSE;
 
void sensorsOn(void)
{
	sensorsEnabled = TRUE;
} 

void sensorsOff(void)
{
	sensorsEnabled = FALSE;
}

void cubeSensorsOn(void)
{
	cubeSensorsEnabled = TRUE;
}

void cubeSensorsOff(void)
{
	cubeSensorsEnabled = FALSE;
}
   
void sensorsInit(void)
{
	sensorsOff();
	// set each sensor line to be an input 
	LEFT_IN_TRIS   = 1;
	FRONT_IN_TRIS  = 1;
	RIGHT_IN_TRIS  = 1;
	LINE_1_IN_TRIS = 1;
	LINE_2_IN_TRIS = 1;
	LINE_3_IN_TRIS = 1;
	LINE_4_IN_TRIS = 1;
	BATTERY_TRIS   = 1;
	// make sure the emitters are outputs
	LEFT_OUT_TRIS	= 0;
	FRONT_OUT_TRIS  = 0;
	RIGHT_OUT_TRIS  = 0;
	LINE_1_OUT_TRIS = 0;
	LINE_2_OUT_TRIS = 0;
	LINE_3_OUT_TRIS = 0;
	LINE_4_OUT_TRIS = 0;
	adcInit();
}

/*void readLineSensors(void)
{
	if(sensorsEnabled)
	{
		lineSensor[0] = readSensor(LINE_1_CHAN);
		lineSensor[1] = readSensor(LINE_2_CHAN);
		lineSensor[2] = readSensor(LINE_3_CHAN);
		lineSensor[3] = readSensor(LINE_4_CHAN);
		intersection = (lineSensor[0]>LINE_THRESHOLD)
							&(lineSensor[1]>LINE_THRESHOLD)
							&(lineSensor[2]>LINE_THRESHOLD)
							&(lineSensor[3]>LINE_THRESHOLD);
	}
}*/

void readLineSensors(void)
{
	if(sensorsEnabled)
	{
		lineSensor[0] = 1024-readSensor(LINE_1_CHAN);
		lineSensor[1] = 1024-readSensor(LINE_2_CHAN);
		lineSensor[2] = 1024-readSensor(LINE_3_CHAN);
		lineSensor[3] = 1024-readSensor(LINE_4_CHAN);
		intersection = (lineSensor[0]>LINE_THRESHOLD)
							&(lineSensor[1]>LINE_THRESHOLD)
							&(lineSensor[2]>LINE_THRESHOLD)
							&(lineSensor[3]>LINE_THRESHOLD);
	}
}

void readCubeSensors(void)
{
	if(sensorsEnabled&&cubeSensorsEnabled)
	{
		frontSensor	  = readSensor(FRONT_CHAN);
		leftSensor    = readSensor(LEFT_CHAN);
		rightSensor   = readSensor(RIGHT_CHAN);
		rightCube = (rightSensor > RIGHT_THRESHOLD);
		leftCube  = (leftSensor > LEFT_THRESHOLD);
		frontCube = (frontSensor > FRONT_THRESHOLD);
	}
}
void readAllSensors(void)//160uS+21uS
{
	if (sensorsEnabled)
	{
		lineSensor[0] = readSensor(LINE_1_CHAN);//*14/10;
		lineSensor[1] = readSensor(LINE_2_CHAN);
		lineSensor[2] = readSensor(LINE_3_CHAN);
		lineSensor[3] = readSensor(LINE_4_CHAN);
		intersection = (lineSensor[0]>LINE_THRESHOLD)
							&(lineSensor[1]>LINE_THRESHOLD)
							&(lineSensor[2]>LINE_THRESHOLD)
							&(lineSensor[3]>LINE_THRESHOLD);
		if(cubeSensorsEnabled)
		{
			frontSensor	  = readSensor(FRONT_CHAN);
			leftSensor    = readSensor(LEFT_CHAN);
			rightSensor   = readSensor(RIGHT_CHAN);
			rightCube = (rightSensor > RIGHT_THRESHOLD);
			leftCube  = (leftSensor > LEFT_THRESHOLD);
			frontCube = (frontSensor > FRONT_THRESHOLD);
		}
	}
}

int readSensor(int sensor)
{
	int temp;
	switch (sensor)
	{
	    case LINE_1_CHAN :
	    	adcSetChannel(LINE_1_CHAN);
	    	__delay_us(7);
	    	temp = adcRead();
			LINE_1_OUT = 1;              // turn on the emitter
			__delay_us(EMITTER_ON_TIME); // wait for the sensor rise-time
			temp = adcRead()-temp;
			LINE_1_OUT = 0;          // turn off the emitter
			break;
			
	    case LINE_2_CHAN :
	    	adcSetChannel(LINE_2_CHAN);
	    	__delay_us(7);
	    	temp = adcRead();
			LINE_2_OUT = 1;              // turn on the emitter
			__delay_us(EMITTER_ON_TIME); // wait for the sensor rise-time
			temp = adcRead()-temp;
			LINE_2_OUT = 0;          // turn off the emitter
			break;
			
	    case LINE_3_CHAN :
	    	adcSetChannel(LINE_3_CHAN);
	    	__delay_us(7);
	    	temp = adcRead();
			LINE_3_OUT = 1;              // turn on the emitter
			__delay_us(EMITTER_ON_TIME); // wait for the sensor rise-time
			temp = adcRead()-temp;
			LINE_3_OUT = 0;          // turn off the emitter
			break;
			
	    case LINE_4_CHAN :
	    	adcSetChannel(LINE_4_CHAN);
	    	__delay_us(7);
	    	temp = adcRead();
			LINE_4_OUT = 1;              // turn on the emitter
			__delay_us(EMITTER_ON_TIME); // wait for the sensor rise-time
			temp = adcRead()-temp;
			LINE_4_OUT = 0;          // turn off the emitter
			break;
			
	    case FRONT_CHAN :
	    	adcSetChannel(FRONT_CHAN);
	    	__delay_us(7);
	    	temp = adcRead();
			FRONT_OUT = 1;              // turn on the emitter
			__delay_us(EMITTER_ON_TIME); // wait for the sensor rise-time
			temp = adcRead()-temp;
			FRONT_OUT = 0;          // turn off the emitter
			break;
			
	    case RIGHT_CHAN :
	    	adcSetChannel(RIGHT_CHAN);
	    	__delay_us(7);
	    	temp = adcRead();
			RIGHT_OUT = 1;              // turn on the emitter
			__delay_us(EMITTER_ON_TIME); // wait for the sensor rise-time
			temp = adcRead()-temp;
			RIGHT_OUT = 0;          // turn off the emitter
			break;
		
		case LEFT_CHAN :
	    	adcSetChannel(LEFT_CHAN);
	    	__delay_us(7);
	    	temp = adcRead();
			LEFT_OUT = 1;              // turn on the emitter
			__delay_us(EMITTER_ON_TIME); // wait for the sensor rise-time
			temp = adcRead()-temp;
			LEFT_OUT = 0;          // turn off the emitter
			break;
			
		default:
			temp = -1;
	}
	
	return temp;
}

unsigned int readBatteryVoltage(void)
{
  batteryVoltage = (adcReadChannel(BATTERY_CHAN) * KBATT) / 512;
  if(batteryVoltage < (33*3))
	{
		motorsOff();
		stopSystemTimer();
		while(TRUE)
		{
			LED_TOGGLE;
			__delay_ms(100);
		}
	}
  return batteryVoltage;	//returns battery vol. Ex. 12.6V=126
}

void printSensors(void)
{
	cubeSensorsOn();
	readLineSensors();
	readCubeSensors();
	putsUART("\r\n\r\n");
	sprintf(outBuf,"\r\nLine  %3d   %3d   %3d   %3d",
											lineSensor[0],
											lineSensor[1],
											lineSensor[2],
											lineSensor[3]);
	putsUART(outBuf);
	if(intersection)
		putsUART("\r\n**************************");
	sprintf(outBuf,"\r\nCube   %3d     %3d     %3d",
											leftSensor,
											frontSensor,
											rightSensor);
	putsUART(outBuf);
	putsUART("\r\n-----------------------------");
} 
