#include "antz.h"
#include "libpic30.h"
#include "systemtimer.h"
#include "buttons.h"
#include "sensors.h"
#include "motors.h"
#include "profiler.h"
#include "pid.h"
#include <stdio.h>
#include "uart.h"
#include "spi.h"
#include "CC2500.h"
#include "grid.h"

volatile unsigned long tickCount;
volatile unsigned int millisecondCount;
volatile unsigned int maxTimer=0;
volatile unsigned int dataUpdated=FALSE;

/*****************************************************
 *  Timer 5 is used to provide the main system clock
 *  It generates an interrupt every millisecond
 *
 *  This ISR is where most of the actual work gets done.
 *  with the sensors running and the straights profiler
 *  active, this interrupt takes about 220us of which
 *  120us is processing the sensors
 *****************************************************/
void _ISR SYSTIM_INTERRUPT(void)
{
	int pidOut;
	unsigned char rxBytes;
	/* reset the interrupt flag */
	SYSTIM_IF = 0;
	//LED_ON;
	tickCount++;      
	millisecondCount--;
	commCount++;
	
	if(!GDO0)
	{
		rxBytes = CC2500_receive_packet();
		if(rxBytes>PACKET_LEN)
		{
			CC2500_idle_mode();
			CC2500_clear_rx_fifo();
			CC2500_clear_tx_fifo();
			CC2500_receive_mode();
			commEnabled = FALSE;
		}
		else
			commEnabled = TRUE;
		if(newPacket)
		{
			deassamble_packet();
			getFellowCoveredSqrs();
			my.obzClear = OBZ_clear();
			if(inOBZ(fellow.location))
			{
				LED_ON;
			}
			else
			{
				LED_OFF;
			}
			dataUpdated = TRUE;
			newPacket = FALSE;
			noCommCount = 0;
		}
		else
			noCommCount++;
	}
	if((commCount>=6)&&commEnabled)
	{
		assamble_packet();
		if(!GDO0)
		{
			CC2500_transmit_packet();
			commCount = 0;
		}
	}
	else if(commCount==4)
	{
		CC2500_idle_mode();
		__delay_us(1);
		CC2500_clear_rx_fifo();
		__delay_us(1);
		CC2500_clear_tx_fifo();
		__delay_us(1);
		CC2500_receive_mode();
	}
	if(!(tickCount&1))
		readCubeSensors();
	
	if(tickCount>300000L)
	{
		motorsOff();
		sensorsOff();
		stopSystemTimer();
		LED_ON;
	}
	doButtons();  
	readLineSensors();
	readCounters();
	doProfiler();
	
	pidOut = doPID( &left_PID_param);
	if( pidOut < -MOTORS_MAX_DC )
		pidOut = -MOTORS_MAX_DC;
	else if( pidOut > MOTORS_MAX_DC )
		pidOut = MOTORS_MAX_DC;
	motorsLeftSetDutyCycle(pidOut);
	
	pidOut = doPID( &right_PID_param);
	if( pidOut < -MOTORS_MAX_DC )
		pidOut = -MOTORS_MAX_DC;
	else if( pidOut > MOTORS_MAX_DC )
		pidOut = MOTORS_MAX_DC;
	motorsRightSetDutyCycle(pidOut);
	
	//LED_OFF;
	
}

/**********************************************************************/
/*  System Timer Initialisation                                       */
/*                                                                    */
/*  As soon as it is initialised, the system timer is also    enabled */
/**********************************************************************/
void systemTimerInit()
{
  stopSystemTimer();
  SYSTIM_PR = SYSTIM_PR_COUNT;
  SYSTIM_IF = 0;    // clear IF bit 
  SYSTIM_IP = 6;    // assigning Interrupt Priority to high 
  SYSTIM_IE = 1;    // Interrupt Enable 
  tickCount = 0;
  millisecondCount = 0;
  //startSystemTimer();
}
void  startSystemTimer(void)
{
  SYSTIM_TON = 1;    
}

void  stopSystemTimer(void)
{
  SYSTIM_TON = 0;    
}

void delayMs(unsigned int ms)
{
	millisecondCount = ms;
	while(ms);
}

int waitDataUpdate(void)
{
	unsigned long temp_tickCount;
	unsigned int i;
	dataUpdated = FALSE;
	for(i=0; i<20; i++)
	{
		temp_tickCount = tickCount;
		while( tickCount == temp_tickCount );
		if(dataUpdated)
			return TRUE;
	}
	return FALSE;
}

void waitToTick(void)
{
	unsigned long temp_tickCount;
	temp_tickCount = tickCount;
		while( tickCount == temp_tickCount );
}

void waitBeforTransmit(void)
{
	unsigned int i;
	
	for(i=0; i<20; i++)
	{
		waitToTick();
		if(commCount==1)
			return;
	}
}
