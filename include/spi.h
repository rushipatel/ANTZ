
#ifndef SPI_H
#define SPI_H


/************************************************************************
* Error and Status Flags                                                *
* SPIM_STS_WRITE_COLLISION indicates that, Write collision has occurred *
* while trying to transmit the byte.                                    *
*                                                                       *    
* SPIM_STS_TRANSMIT_NOT_OVER indicates that, the transmission is        *
* not yet over. This is to be checked only when non Blocking            *
* option is opted.                                                      *
*                                                                       *    
* SPIM_STS_DATA_NOT_READY indicates that reception SPI buffer is empty  *
* and there's no data avalable yet.                                     * 
*                                                                       *    
************************************************************************/

#define SPI_OK  0
#define SPI_WRITE_COLLISION    1
#define SPI_NOT_OVER  2  
#define SPI_DATA_NOT_READY     3  

/************************************************************************
* Macro: SPI_getc
*                                                                       *
* PreCondition: 'SPIMPolIsTransmitOver' should return a '0'.            *
* Overview: This macro reads a data received                            *
* Input: None                                                           *
* Output: Data received                                                 *
*                                                                       *
************************************************************************/
#define  SPI_getc() SPIBUF

extern void SPI_init();
extern unsigned char SPI_putc(unsigned char Data);

#define enable_spi() SPI1STATbits.SPIEN = 1
#define disable_spi() SPI1STATbits.SPIEN = 0

#endif
