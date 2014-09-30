#ifndef ADC_H
#define ADC_H

#ifndef ADC_SPEED
#define ADC_SPEED 800000UL
#endif

/***********************************************
 *  
 *  ADC module for the dsPIC24/30/33 ()
 * part of decimus 2224
 *
 ***********************************************/
void adcInit(void);
 
/*********************************************************************
* Function Name     : adcReadChannel
* Description       : Does a single conversion for one channel
* Parameters        : None
* Return Value      : ADC result
*********************************************************************/
unsigned int adcReadChannel (unsigned int chan);

/*********************************************************************
* Function Name     : adcStartConversion
* Description       : This function starts an A/D conversion by 
*                     clearing ADCON1<SAMP> bit.
* Parameters        : None
* Return Value      : None
*********************************************************************/
void adcStartConversion(void);

/*********************************************************************
* Function Name     : adcClose
* Description       : This function turns off the A/D converter. 
*                     Also, the Interrupt enable (ADIE) and Interrupt 
*                     flag (ADIF) bits are cleared
* Parameters        : None
* Return Value      : None
*********************************************************************/
void adcClose(void);

/*********************************************************************
* Function Name     : adcBusy
* Description       : This function returns the ADC conversion status
* Parameters        : None
* Return Value      : DONE bit status
*********************************************************************/
char adcBusy(void);

/*********************************************************************
* Function Name     : adcSetChannel
* Description       : This function sets the AD1CHS reg 
*					  and starts sampling
* Parameters        : unsigned channel
* Return Value      : None
*********************************************************************/
void adcSetChannel(unsigned int channel);

/*********************************************************************
* Function Name     : adcRead
* Description       : This function stops sampling and trriger
*					: the conversion, requires acquisition done
* Parameters        : None
* Return Value      : ADC result
*********************************************************************/
unsigned int adcRead(void);

#endif
