#include <stdio.h>
#include "antz.h"
#include <libpic30.h>		//for delay routines
#include "servo.h"
#include "buttons.h"
#include "systemtimer.h"
#include "spi.h"
#include "CC2500.h"
#include "motors.h"
#include "uart.h"
#include "sensors.h"
#include "pid.h"
#include "profiler.h"
#include "grid.h"

#ifdef MACHINE_B
	#warning "This machine is B"
#elif defined MACHINE_A
	#warning "This machine is A"
#else
	#error "Machine type is not defined"
#endif

/**********************************************************************/
/*Device configuration register macros for building the hex file      */
_FOSCSEL(FNOSC_FRC);			  				 /* Select Internal FRC at POR */
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & IOL1WAY_OFF); /*Enable Clock Switching and IO PIN*/
_FWDT(FWDTEN_OFF);                           /*Watchdog timer disabled*/
_FPOR(FPWRT_PWR128 & PWMPIN_ON);							    /* POR Disable */
/**********************************************************************/

void statusInitA(void);
void statusInitB(void);

unsigned int displayEnabled = 0;
char outBuf[50]={NULL};
unsigned int rx_buff_temp[PACKET_LEN];
unsigned char cmlInd = 0;
volatile STATUS my,fellow,st_temp;
MACHINE_STATE mcState,mcNextState;

void PLLConfig(void)
{
	PLLFBD = 78; 						// M = 80
	CLKDIVbits.PLLPOST = 0;			// N2 = 2
	CLKDIVbits.PLLPRE = 3; 			// N1 = 5
	
	// Initiate Clock Switch to Internal FRC with PLL (NOSC = 0b001)
	__builtin_write_OSCCONH(0x01);	  // Selects the new Oscillator,FRCPLL
	__builtin_write_OSCCONL(0x01);	  // Request Osc switch to FRCPLL
	while (OSCCONbits.COSC != 0b001); // Wait for Clock switch to occur
	while(OSCCONbits.LOCK!=1);	 	  // Wait for PLL to lock
}

void hardwareInit(void)
{
	PLLConfig();
	SET_ADPCFG;
	buttonsInit();
	LED_SET_OUT; 
	LED_OFF;
	sensorsInit();
	sensorsOn();
	readBatteryVoltage();
	servoInit();
	motorsInit();
	//motorsOff();
	profilerInit();
	readButtons();
	if( key_UP&&key_DN ) //if ICSP is connected
	{
		displayEnabled = TRUE;
		serialInit();
		//_TRISB10 = 0;
		//_TRISB11 = 0;
	}
	CC2500_init();
	systemTimerInit();
}
		
int resetSource(void)
{
	if (EXTR) 
		return EXTERNAL_RESET;
	if (POR) 
		return POWER_ON_RESET;
	if (BOR) 
		return BROWN_OUT_RESET;
	if (WDTO) 
		return WATCHDOG_RESET;
	if (SWR) 
		return SOFTWARE_RESET;
	if (IOPUWR) 
		return EXCEPTION_RESET;
	if (TRAPR)
		return TRAP_RESET;
	return UNKNOWN_RESET;
}


int main(void)
{
	unsigned char temp_sqr,temp_dir,temp_diff,temp;
	unsigned int i;
	unsigned char channel=150;
	hardwareInit();
	//FOLD_ARMS;
	/*__delay_ms(50);
	do{
		if(key_UP)
		{
			channel+=10;
			LED_ON;
			__delay_ms(20);
			LED_OFF;
		}
		if(key_DN)
		{
			channel-=10;
			LED_ON;
			__delay_ms(20);
			LED_OFF;
		}
		__delay_ms(300);
		readButtons();
	}	while(!key_GO);
	LED_ON;
	__delay_ms(200);
	LED_OFF;
	CC2500_init();
	CC2500_write_register(CHANNR,channel);
	CC2500_idle_mode();
	CC2500_receive_mode();
	do{
		if(key_UP)
		{
			speed++;
			if(speed>4)
				speed = 4;
			LED_ON;
			__delay_ms(20);
			LED_OFF;
		}
		if(key_DN)
		{
			speed--;
			if(speed<0)
				speed = 0;
			LED_ON;
			__delay_ms(20);
			LED_OFF;
		}
		__delay_ms(300);
		readButtons();
	}while(!key_GO);*/
	LED_ON;
	__delay_ms(150);
	LED_OFF;
	__delay_ms(150);
	LED_ON;
	__delay_ms(150);
	LED_OFF;
	
	#ifdef MACHINE_A
		statusInitA();
	#endif
	#ifdef MACHINE_B
		statusInitB();
	#endif
	assamble_packet();
	CC2500_idle_mode();
	CC2500_clear_rx_fifo();
	CC2500_clear_tx_fifo();
	CC2500_receive_mode();
	__delay_ms(100);
	i = CC2500_read_status_register( RXBYTES);
	i&=0xFF;
	sprintf(outBuf,"\r\n%u\r\n",i);
	putsUART(outBuf);
	if(i)
	{
		CC2500_idle_mode();
		CC2500_clear_rx_fifo();
		CC2500_clear_tx_fifo();
		CC2500_receive_mode();
		while(GDO0==0);
		while(GDO0==1);
		SYSTIM_TMR = 9581;
		__delay_ms(3);
		commCount = 0;
		startSystemTimer();
		noCommCount=0;
		commEnabled = TRUE;
		if(waitDataUpdate())
			my.obzEntrCnt = fellow.obzEntrCnt;
		/*LED_ON;
		__delay_ms(2000);*/
		
	}
	else
	{
		startSystemTimer();
		noCommCount=0;
		commEnabled = TRUE;
	}
	#ifdef MACHINE_A
	while(!key_GO){
		if(noCommCount>100)
		{
			commEnabled = FALSE;
			waitToTick();
			CC2500_init();
			commEnabled = TRUE;
			__delay_ms(40);
			//noCommCount = 0;
		}
	}
	#endif

	if(displayEnabled)
	{
		switch(resetSource())
		{
			case POWER_ON_RESET:
				sprintf(outBuf,"\r\nBattry Voltage : %u",batteryVoltage);
				putsUART(outBuf);
				break;
			
			case EXTERNAL_RESET:
				sprintf(outBuf,"\r\nUser Reset...");
				putsUART(outBuf);
				break;
			
			default:
				sprintf(outBuf,"\r\nUnknown Reset");
				putsUART(outBuf);
				break;
		}
	}
	/*while(TRUE)
	{
		printSensors();
		__delay_ms(500);
	}*/
	
	gridInit();
	#ifdef MACHINE_B
	while(TRUE)
	{
		if(noCommCount>100)
		{
			commEnabled = FALSE;
			waitToTick();
			CC2500_init();
			commEnabled = TRUE;
			__delay_ms(40);
			//noCommCount = 0;
		}
		if(dataUpdated)
		{
			printStatus(&fellow);
			/*rx_buff_temp[0] = rx_buff0;
			rx_buff_temp[1] = rx_buff1;
			rx_buff_temp[2] = rx_buff2;
			rx_buff_temp[3] = rx_buff3;
			rx_buff_temp[4] = rx_buff4;
			rx_buff_temp[5] = rx_buff5;
			sprintf(outBuf,"\r\n\r\n%u,%u,%u,%u,%u,%u",
												rx_buff_temp[0],
												rx_buff_temp[1],
												rx_buff_temp[2],
												rx_buff_temp[3],
												rx_buff_temp[4],
												rx_buff_temp[5]);
			putsUART(outBuf);*/
			dataUpdated = FALSE;
		}		
	}
	//while(!key_GO);
	while(fellow.heading==10&&!key_GO);
	//while(fellow.heading==10);
	#endif
	
	while(TRUE)
	{
		if(noCommCount>100)
		{
			commEnabled = FALSE;
			waitToTick();
			CC2500_init();
			commEnabled = TRUE;
			__delay_ms(40);
			//noCommCount = 0;
		}
		switch(mcState)
		{
			case MC_START:
			{
				cubeSensorsOn();
				waitToTick();
				waitToTick();
				cubeSensorsOff();
				temp_sqr = nextSquare(my.location,my.heading&0x07);
				grid[temp_sqr] |= VISITED;
				if(frontCube)
				{
					myCube[++cubeInd] = temp_sqr;
					mcState=MC_DELIVER;
				}
				else
					mcState=MC_SCAN;
				break;
			}
			case MC_SCAN:
			{
				temp_sqr = scanMap[my.smInd];
				if(my.location==temp_sqr)
				{
					my.smInd++;
					break;
				}
				my.goalSquare = temp_sqr;
				mcNextState = MC_SCAN;
				mcState = MC_GET_M_THR;
				/*temp_dir = nextDirection(my.location,temp_sqr);
				temp_diff = temp_dir - (my.heading&0x07);
					
				if(prState==PR_STR_ACC||prState==PR_STR_DEC)
				{
					if(temp_diff)
						break;
					waitDataUpdate();
					if(commonSquare(temp_sqr))
						break;
					my.heading &= 0x07;
					finalPos += COUNTS_PER_CELL*256;
					if(prState==PR_STR_DEC)
						prState = PR_STR_ACC;
					while(relativeDistance>distance[ORTHO_SENS_DIST]*2);
				}
				else if(prState==PR_FINISHED)
				{
					if(temp_diff)
					{
						commandList[cmlInd++] = getTurnCmd(temp_diff);
						my.heading = temp_dir | NODIR;
					}
					waitDataUpdate();
					if(commonSquare(temp_sqr))
					{
						commandList[cmlInd++] = CMD_STOP;
						getNextMove();
						break;
					}
					my.heading &= 0x07;
					commandList[cmlInd++] = CMD_STRAIGHT | CMD_FORWARD;
					commandList[cmlInd++] = CMD_STOP;
					getNextMove();
				}
				else
					break;
				while(prState!=PR_STR_ACC);
				while(relativeDistance<distance[ORTHO_SENS_DIST]*2);
				my.location = nextSquare(my.location,my.heading);
				grid[my.location] |= ONROUTE;
				my.heading |= NODIR;
				if(scanNeighbours())
				{
					mcState = MC_DELIVER;
				}*/
				my.smInd++;
				if(my.smInd>=17)
				{
					my.smInd = 0;
					#ifdef MACHINE_A
					my.goalSquare = 20;
					#endif
					#ifdef MACHINE_B
					my.goalSquare = 60;
					#endif
					mcNextState = MC_SCAN;
					mcState = MC_GET_M_THR;
				}
			break;
			}
			case MC_GET_M_THR:
			{
				waitDataUpdate();
				updateGrid();
				floodGrid(my.goalSquare);
				/*if(map[my.location] == 0)
				{
					mcState = mcNextState;
					break;
				}*/
				temp_sqr = nextNeighbour(my.location);
				temp_dir = nextDirection(my.location,temp_sqr);
				temp_diff = temp_dir - (my.heading&0x07);
				waitDataUpdate();
				if(commonSquare(temp_sqr))
				{
					break;
				}
				if((!inOBZ(my.location)) && inOBZ(temp_sqr))
				{
					if(!my.obzClear || !my.delivering)
						break;

				}
				if(inOBZ(my.location) && (!inOBZ(temp_sqr)))
				{
					my.obzEntrCnt+=2;
				}
				if(isDipoSquare(temp_sqr)&& my.delivering)
				{
					mcState = MC_DELIVER_FINISH;
					break;
				}
				if(prState==PR_STR_ACC||prState==PR_STR_DEC)
				{
					if(temp_diff)
						break;
					my.heading &= 0x07;
					finalPos += COUNTS_PER_CELL*256;
					if(prState==PR_STR_DEC)
						prState = PR_STR_ACC;
					while(relativeDistance>distance[ORTHO_SENS_DIST]*2);
				}
				else if(prState==PR_FINISHED)
				{
					if(temp_diff)
					{
						commandList[cmlInd++] = getTurnCmd(temp_diff);
						commandList[cmlInd++] = CMD_STOP;
						getNextMove();
						my.heading = temp_dir|NODIR;
						break;
					}
					else
					{
						if(!(grid[temp_sqr]&VISITED));
						{
							cubeSensorsOn();
							waitToTick();
							waitToTick();
							cubeSensorsOff();
							if(!commonSquare(temp_sqr))
							{
								grid[temp_sqr] |= VISITED;
								if(frontCube)
								{
									updateCube(temp_sqr);
									break;
								}
							}
						}
						my.heading &= 0x07;
						commandList[cmlInd++] = CMD_STRAIGHT | CMD_FORWARD;
						commandList[cmlInd++] = CMD_STOP;
						getNextMove();
					}
				}
				else
					break;
				while(prState!=PR_STR_ACC);
				while(relativeDistance<distance[ORTHO_SENS_DIST]*2);
				my.location = nextSquare(my.location,my.heading);
				grid[my.location] |= ONROUTE;
				my.heading |= NODIR;
				if(scanNeighbours()&&(my.delivering==FALSE))
				{
					mcState = MC_DELIVER;
					break;
				}
				if(map[my.location] == 0)
					mcState = mcNextState;
				break;
			}
			case MC_DELIVER:
			{
				temp_sqr = unDipoCube();
				if(temp_sqr)
				{
					waitDataUpdate();
					if(inOBZ(temp_sqr)&&!my.obzClear)
						break;
					temp_dir = nextDirection(my.location,temp_sqr);
					if(temp_dir==NODIR)
					{
						temp_sqr = nearestNeighbourOfCube();
						if(temp_sqr)
						{
							my.goalSquare = temp_sqr;
							mcNextState = MC_DELIVER;
							mcState = MC_GET_M_THR;
						}
						else
						{
							do{
								my.smInd++;
							}while(grid[scanMap[my.smInd]]&ONROUTE);
							my.goalSquare = scanMap[my.smInd];
							mcNextState = MC_SCAN;
							mcState = MC_GET_M_THR;
						}
						break;
					}
					else
					{
						grabCube(temp_dir);
						if(my.delivering)
						{
							temp = temp_sqr;
							my.goalSquare = my.location;
							temp_sqr = getDipoSquare();
							if(temp_sqr)
							{
								for(i=0; i<4; i++)
								{
									if((myCube[i]<81)&&myCube[i]==temp)
										temp = i;
								}
								
								myCube[/*cubeIndForMcDeliver*/temp] = temp_sqr;
								my.goalSquare = temp_sqr;
								mcNextState = MC_DELIVER;
								mcState = MC_GET_M_THR;
								break;
							}
						}
					}
					break;
				}
				do{
					my.smInd++;
				}while(grid[scanMap[my.smInd]]&ONROUTE);
				my.goalSquare = scanMap[my.smInd];
				mcNextState = MC_SCAN;
				mcState = MC_GET_M_THR;
				break;
			}
			case MC_DELIVER_FINISH:
			{
				deliverCube();
				if(unDipoCube())
				{
					mcState = MC_DELIVER;
					break;
				}
				do{
					my.smInd++;
				}while(grid[scanMap[my.smInd]]&ONROUTE);
				my.goalSquare = scanMap[my.smInd];
				mcNextState = MC_SCAN;
				mcState = MC_GET_M_THR;
			}
		}
	}	
	while(TRUE);
	return 0;
}

void statusInitA(void)
{
	int idx;
	my.heading = 10;
	my.location = 20;
	my.goalSquare = 20;
	my.obzClear = TRUE;
	my.delivering = FALSE;
	my.coveredSqr[0] = 20;
	my.coveredSqr[1] = 29;
	my.coveredSqr[2] = 255;
	my.coveredSqr[3] = 255;
	my.smInd = 0;
	my.obzEntrCnt = 0;
	
	fellow.heading = 14;
	fellow.location = 60;
	fellow.goalSquare = 60;
	fellow.obzClear = TRUE;
	fellow.delivering = FALSE;
	fellow.coveredSqr[0] = 60;
	fellow.coveredSqr[1] = 254;
	fellow.coveredSqr[2] = 254;
	fellow.coveredSqr[3] = 254;
	fellow.obzEntrCnt = 1;
	for(idx=0; idx<17; idx++)
		scanMap[idx]=SCAN_MAP_A[idx];
	cubeInd=0-1;
	mcState = MC_START;
	
	/*my.goalSquare = 4;
	mcState = MC_GET_M_THR;*/
}

void statusInitB(void)
{
	int idx;
	my.heading = 14;
	my.location = 60;
	my.goalSquare = 60;
	my.obzClear = TRUE;
	my.delivering = FALSE;
	my.coveredSqr[0] = 60;
	my.coveredSqr[1] = 51;
	my.coveredSqr[2] = 254;
	my.coveredSqr[3] = 254;
	my.smInd = 0;
	my.obzEntrCnt = 1;
	
	fellow.heading = 10;
	fellow.location = 20;
	fellow.goalSquare = 20;
	fellow.obzClear = TRUE;
	fellow.delivering = FALSE;
	fellow.coveredSqr[0] = 20;
	fellow.coveredSqr[1] = 29;
	fellow.coveredSqr[2] = 255;
	fellow.coveredSqr[3] = 255;
	fellow.obzEntrCnt = 0;
	for(idx=0; idx<17; idx++)
		scanMap[idx]=SCAN_MAP_B[idx];
	cubeInd=0-1;
	mcState = MC_START;
}

void printStatus(STATUS *st)
{
	sprintf(outBuf,"\r\n\r\nDI:%u",st->heading);
	putsUART(outBuf);
	sprintf(outBuf,"\r\nLO:%u",st->location);
	putsUART(outBuf);
	sprintf(outBuf,"\r\nOC:%u",st->obzEntrCnt);
	putsUART(outBuf);
}
