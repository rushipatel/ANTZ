

/***********************************************
 *  
 *  ADC module for the dsPIC24/30/33 ()
 *
 *  This module is heavily hardware dependant
 ***********************************************/
#include "antz.h"
#include "libpic30.h"
#include "adc.h"
 
 /*
  * The adc will be configured for 10bit conversions on demand. 
  * These should be as fast as possible with 7 channels used.
  */
void adcInit(void)
{
	AD1CON1bits.ADON = 0;  // disable the ADC
	AD1CON1bits.AD12B= 0;  // 10-bit mode
	AD1CON1bits.FORM = 0;  // dont sign extended output
	AD1CON1bits.SSRC = 0;  // Clearing SAMP bit ends sampling and starts conversion
	AD1CON1bits.ASAM = 0;  // sampling begins when SAMP is set
	AD1CON3bits.ADCS = (FCY/ADC_SPEED/12)-1;
	AD1CON1bits.ADON = 1;  // enable the ADC    
}

 unsigned int adcReadChannel (unsigned int chan)
{
	AD1CHS0 = chan;
	AD1CON1bits.SAMP = 1; // start sampling
	__delay_us(1);
	AD1CON1bits.SAMP = 0; // start Converting
	while (!AD1CON1bits.DONE); // conversion done?
	return ADC1BUF0;
}



/*********************************************************************
* Function Name     : startConversion
* Description       : This function starts an A/D conversion by 
*                     clearing ADCON1<SAMP> bit.
* Parameters        : None
* Return Value      : None
*********************************************************************/

void adcStartConversion(void)
{
     AD1CON1bits.SAMP = 0; /* clear SAMP to start conversion*/
}

/*********************************************************************
* Function Name     : CloseADC10
* Description       : This function turns off the A/D converter. 
*                     Also, the Interrupt enable (ADIE) and Interrupt 
*                     flag (ADIF) bits are cleared
* Parameters        : None
* Return Value      : None
*********************************************************************/

void adcClose(void)
{
    AD1CON1bits.ADON = 0;   
}


/*********************************************************************
* Function Name     : BusyADC10
* Description       : This function returns the ADC conversion status.
* Parameters        : None
* Return Value      : DONE bit status
*********************************************************************/

char adcBusy(void)
{
    return !(AD1CON1bits.DONE);	/* returns the DONE bit status */
}

/*********************************************************************
* Function Name     : adcSetChannel
* Description       : This function sets the AD1CHS reg 
*					  and starts sampling
* Parameters        : unsigned channel
* Return Value      : None
*********************************************************************/

void adcSetChannel(unsigned int channel)
{
    AD1CHS0 = channel;
   	AD1CON1bits.SAMP = 1; // start sampling
}

/*********************************************************************
* Function Name     : adcRead
* Description       : This function stops sampling and trriger
*					: the conversion, requires acquisition done
* Parameters        : None
* Return Value      : ADC result
*********************************************************************/
unsigned int adcRead(void)
{
	AD1CON1bits.SAMP = 0; // start Converting
	while (!AD1CON1bits.DONE); // conversion done?
	AD1CON1bits.SAMP = 1; // start sampling again
	return ADC1BUF0;
}
