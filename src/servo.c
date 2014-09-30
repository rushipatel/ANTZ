#include "antz.h"
#include "servo.h"

void servoInit(void)
{
		//OC1 Config
	OC1CONbits.OCM = 0b000;		// Disable Output Compare Module
	OC1R = 0;						// Write the duty cycle for the first PWM pulse
	OC1RS = 0;						// Write the duty cycle for the second PWM pulse
	OC1CONbits.OCTSEL = 0; 		// Select Timer 2 as output compare time base
	OC1R = 0; 						// Load the Compare Register Value
	OC1CONbits.OCM = 0b110; 	// Select the Output Compare mode
	
		//OC2 Config
	OC2CONbits.OCM = 0b000;		// Disable Output Compare Module
	OC2R = 0;						// Write the duty cycle for the first PWM pulse
	OC2RS = 0;						// Write the duty cycle for the second PWM pulse
	OC2CONbits.OCTSEL = 0; 		// Select Timer 2 as output compare time base
	OC2R = 0; 						// Load the Compare Register Value
	OC2CONbits.OCM = 0b110; 	// Select the Output Compare mode
	
		// Initialize and enable Timer2
	T2CONbits.TON = 0; 			// Disable Timer
	T2CONbits.TCS = 0; 			// Select internal instruction cycle clock
	T2CONbits.TGATE = 0; 		// Disable Gated Timer mode
	T2CONbits.TCKPS = 0b01; 	// Select 1:8 Prescaler
	TMR2 = 0x00; 					// Clear timer register
	PR2 = 0xFFFF;					// 65535=56Hz for 29.48M Fcy & 1:8 Prescaler
	T2CONbits.TON = 1; 			// Start Timer
	
		// IO Selection
	__builtin_write_OSCCONL(OSCCON & ~(1<<6));// IO Unlock Registers
	RPOR12bits.RP25R = 0b10011;					// RP25 as O/P for OC2
	RPOR11bits.RP23R = 0b10010;					// RP24 as O/P for OC1
	__builtin_write_OSCCONL(OSCCON | (1<<6));	// IO Lock Registers
}

/***************************************************************/
/* setServoPos() function sets the servo position by changing	*/
/*	dutycycle of Output Compare module									*/
/*	argument's value is between 0-180 degree						  	*/
/***************************************************************/ 	
void setServoPos(unsigned long servoLeftPos,unsigned long servoRightPos)
{
	SERVO_LEFT_DCR = SERVO_MIN + (servoLeftPos*DIFF_DC/180);
	SERVO_RIGHT_DCR = SERVO_MIN + (servoRightPos*DIFF_DC/180);
}
