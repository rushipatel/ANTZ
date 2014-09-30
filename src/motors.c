#include <math.h>
//#include <limits.h>
#include "antz.h"
#include "motors.h"
#include "pid.h"
#include "profiler.h"
#include "buttons.h"
#include "uart.h"

PID_STRUCT left_PID_param, right_PID_param;

volatile signed int leftCount,rightCount,oldLeftCount,oldRightCount;
volatile signed int leftPosCountH=0, rightPosCountH=0;
volatile signed long old_left_CV,old_right_CV;

void motorsInit(void)
{
	/*LEFT_MOTOR_L  = 1;
	LEFT_MOTOR_H  = 1;
	RIGHT_MOTOR_L = 1;
	RIGHT_MOTOR_H = 1;
	LEFT_MOTOR_L_TRIS  = 0;
	LEFT_MOTOR_H_TRIS  = 0;
	RIGHT_MOTOR_L_TRIS = 0;
	RIGHT_MOTOR_H_TRIS = 0;*/
	
	P1TCONbits.PTMOD = 0b00;	//Free Running mode
	P1TCONbits.PTCKPS = 0b00;	//input prescale 1:1
	P1TCONbits.PTOPS = 0b00;	//output post scale 1:1
	P1TPER = (FCY/FPWM)-1;		//Fcy=29.48 --> 10kHz

	PWM1CON1bits.PMOD1 = 0;		//PWM I/O pair in complementary
	PWM1CON1bits.PMOD2 = 0;		//PWM I/O pair in complementary
	PWM1CON1bits.PMOD3 = 0;		//PWM I/O pair in complementary
	PWM1CON1bits.PEN1H = 1;		//Pin controlled by PWM Module
	PWM1CON1bits.PEN2H = 1;		//Pin controlled by PWM Module
	PWM1CON1bits.PEN3H = 0;		//Pin is not controlled by module
	PWM1CON1bits.PEN1L = 1;		//Pin controlled by PWM Module
	PWM1CON1bits.PEN2L = 1;		//Pin controlled by PWM Module
	PWM1CON1bits.PEN3L = 0;		//Pin is not controlled by module
	PWM1CON2bits.UDIS  = 0;		//Update is enabled 
	PWM1CON2bits.IUE = 1;		//Immediate update of PWM enabled 

	/* PWM I/O pin controlled by PWM Generator */
	P1OVDCONbits.POVD2H = 1;
	P1OVDCONbits.POVD1H = 1;
	P1OVDCONbits.POVD2L = 1;
	P1OVDCONbits.POVD1L = 1;
	
	/* Initialize duty cycle values for PWM1, PWM2 and signals */
	P1DC1 = 1473;
	P1DC2 = 1473;
	
	P1TCONbits.PTEN = 1;
	
	/*Remapable pin assignment*/
	__builtin_write_OSCCONL(OSCCON & ~(1<<6));	//Unlock
	RPINR14bits.QEB1R = 5;
	RPINR14bits.QEA1R = 1;
	RPINR16bits.QEB2R = 7;
	RPINR16bits.QEA2R = 6;
	__builtin_write_OSCCONL(OSCCON | (1<<6));	//Lock again
	
	QEI1CONbits.PCDOUT  = 0;	//Direction output disable
	QEI1CONbits.SWPAB	= QEI1_SWAP_AB;
	POS1CNT = 0;
	MAX1CNT = 0xFFFF;
	QEI1CONbits.QEIM    = 0b101;//QEI 2x Mode
	
	QEI2CONbits.PCDOUT  = 0;	//Direction output disable
	QEI2CONbits.SWPAB	= QEI2_SWAP_AB;
	POS2CNT = 0;
	MAX2CNT = 0xFFFF;
	QEI2CONbits.QEIM    = 0b101;//QEI 2x Mode
	
	IPC14bits.QEI1IP = 7;
	IPC18bits.QEI2IP = 7;
	IFS3bits.QEI1IF  = 0;
	IFS4bits.QEI2IF  = 0;
	IEC3bits.QEI1IE  = 1;
	IEC4bits.QEI2IE  = 1;
	
	leftCount	 = 0;
	rightCount	 = 0;
	old_left_CV	 = 0;
	old_right_CV = 0;

	/*Left motor initial pid parameters*/
	left_PID_param.setValue     = 0;
	left_PID_param.currentValue = 0;
	left_PID_param.Kp = MOTORS_KP;
	left_PID_param.Kd = MOTORS_KD;
	left_PID_param.Ki = MOTORS_KI;
	left_PID_param.Ko = MOTORS_KO;
	left_PID_param.prevErr = 0;
	left_PID_param.Ierror  = 0;

	/*Right motor initial pid parameters*/
	right_PID_param.setValue     = 0;
	right_PID_param.currentValue = 0;
	right_PID_param.Kp = MOTORS_KP;
	right_PID_param.Kd = MOTORS_KD;
	right_PID_param.Ki = MOTORS_KI;
	right_PID_param.Ko = MOTORS_KO;
	right_PID_param.prevErr = 0;
	right_PID_param.Ierror  = 0;
}

void motorsRightSetDutyCycle(int dutyCycle)
{
	P1DC2 = (dutyCycle + MOTORS_MAX_DC)<<1;
}

void motorsLeftSetDutyCycle(int dutyCycle)
{
	P1DC1 = (dutyCycle + MOTORS_MAX_DC)<<1;
}

void motorsOn(void)
{
	P1TCONbits.PTEN = 1;
}

void motorsOff(void)
{
	P1TCONbits.PTEN = 0;
}

void motorsUpdateSpeed(void)
{
	left_PID_param.setValue += (currentSpeed + diffSpeed);
	right_PID_param.setValue += (currentSpeed - diffSpeed);
}
 
void readCounters(void)
{
	unsigned int t1L,t2L;
	signed int t1H,t2H;
	
	t1H = leftPosCountH;
	t1L = POS1CNT;
	t2H = rightPosCountH;
	t2L = POS2CNT;
	
	left_PID_param.currentValue  = ( (long)t1H<<16 ) + t1L;
	right_PID_param.currentValue = ( (long)t2H<<16 ) + t2L;

	leftCount  = left_PID_param.currentValue - old_left_CV;
		if(leftCount<0){leftCount *= -1;}
	rightCount = right_PID_param.currentValue - old_right_CV;
		if(rightCount<0){rightCount *= -1;}

	if ( leftCount > 400 )				//about 3m/s, robo can't archive
	{
		leftCount  = oldLeftCount;
		left_PID_param.currentValue = left_PID_param.setValue;
	}
	
	if ( rightCount > 400 )				//about 3m/s, robo can't archive
	{
		rightCount = oldRightCount;
		right_PID_param.currentValue = right_PID_param.setValue;
	}
	oldLeftCount  = leftCount;
	oldRightCount = rightCount;
	old_left_CV   = left_PID_param.currentValue;
	old_right_CV  = right_PID_param.currentValue;
}

void __attribute__((interrupt, auto_psv)) _QEI1Interrupt(void)
{
	if(QEI1CONbits.UPDN)
		leftPosCountH++;
	else
		leftPosCountH--;
		
	IFS3bits.QEI1IF=0;
}

void __attribute__((interrupt, auto_psv)) _QEI2Interrupt(void)
{
	if(QEI2CONbits.UPDN)
		rightPosCountH++;
	else
		rightPosCountH--;

	IFS4bits.QEI2IF=0;
}
