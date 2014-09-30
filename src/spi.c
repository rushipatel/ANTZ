/********************************************************************/
/*  SPI port driver for dsPIC										*/ 
/*  derived from Microchip generic code for the dsPIC/PIC24 family	*/
/********************************************************************/
#include "antz.h"
#include "spi.h"

/*
 *  function:  SPI_init
 *  before use, be sure to set the relevant port pins to:
 *    SCK : output
 *    SDO : output
 *    SDI : input
 *
 */
	
void SPI_init(void)
{
	SPI1STAT = 0;
	SPI1CON1bits.DISSCK = 0;	//Internal Serial Clock is Enabled.
	SPI1CON1bits.DISSDO = 0;	//SDOx pin is controlled by the module.
	SPI1CON1bits.MODE16 = 0;	//Communication is 8bit-wide.
	SPI1CON1bits.SMP = 0; 	 	//Input Data is sampled at the middle of data output time.
	SPI1CON1bits.CKE = 0; 	 	//Serial output data changes on transition from
	SPI1CON1bits.CKP = 0; 	 	//Idle state for clock is a low level;
	SPI1CON1bits.MSTEN = 1;  	//Master Mode Enabled
	SPI1CON1bits.PPRE = 0b10;	//Primery prescaler 4:1,
	SPI1CON1bits.SPRE = 0b001; 	//secondry 7:1 (29.48/4*7)
	
	//Remapable IO assisgnment
	__builtin_write_OSCCONL(OSCCON & ~(1<<6));	//Unlock
	RPINR20bits.SDI1R = SDI1_R;
	SDO1RPnR = 0b00111;
	SCK1RPnR = 0b01000;
	__builtin_write_OSCCONL(OSCCON | (1<<6));	//Lock again
	
	//SPI1STATbits.SPIEN = 1;  	//Enable SPI Module
}

/*
 *  function SPI_putc
 *  prerequisites:  SPI port initialised with SPI_init()
 */
    
unsigned char SPI_putc(unsigned char Data)
{ 
	IFS0bits.SPI1IF = 0;
	SPI1BUF = Data;
	while(!IFS0bits.SPI1IF);  	// Wait until opcode/constant is transmitted.
	return SPI1BUF;         // must do this read or get an overflow next time
}
