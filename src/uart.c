#include "antz.h"
#include "uart.h"

void serialInit(void)
{
	//_TRISB10 = 0;
	__builtin_write_OSCCONL(OSCCON & ~(1<<6));	//Unlock
	RPOR5bits.RP10R   = 0b00011;
	RPINR18bits.U1RXR = 11;
	__builtin_write_OSCCONL(OSCCON | (1<<6));	//Lock again
	
	U1MODEbits.UARTEN = 0;  // disable the uart
	U1MODEbits.PDSEL = 0;   // 8 bits, no parity
	U1MODEbits.STSEL = 0;   // one stop bit	
	U1BRG = 47;			  	//=(FCY/16/57578)-1, 57578baud
	U1MODEbits.UARTEN = 1;  // enable the uart
	U1STAbits.UTXEN = 1;	// enable transmission
}



int putUART(char c)
{
	while (U1STAbits.UTXBF);  // wait for the transmit buffer to empty
	U1TXREG = c;
	return 0;
}

int getUART(void)
{
	while(!U1STAbits.URXDA);
	return(U1RXREG);
}

int putsUART(char * s)
{
	while(*s)
	{
		putUART(*s);
		s++;
	}
	return 0;
}
