#include "antz.h"
#include "spi.h"
#include <libpic30.h>
#include "CC2500.h"
#include "grid.h"
#include "profiler.h"
#include "uart.h"

const unsigned char CC2500_settings[0x2F]=
{
	CC2500_IOCFG2_value,
	CC2500_IOCFG1_value,	
	CC2500_IOCFG0D_value,
	CC2500_FIFOTHR_value,
	CC2500_SYNC1_value,
	CC2500_SYNC0_value,
	CC2500_PKTLEN_value,
	CC2500_PKTCTRL1_value,
	CC2500_PKTCTRL0_value,
	CC2500_ADDR_value,
	CC2500_CHANNR_value,
	CC2500_FSCTRL1_value,
	CC2500_FSCTRL0_value,
	CC2500_FREQ2_value,
	CC2500_FREQ1_value,
	CC2500_FREQ0_value,
	CC2500_MDMCFG4_value,
	CC2500_MDMCFG3_value,
	CC2500_MDMCFG2_value,
	CC2500_MDMCFG1_value,
	CC2500_MDMCFG0_value,
	CC2500_DEVIATN_value,
	CC2500_MCSM2_value,
	CC2500_MCSM1_value, 
	CC2500_MCSM0_value,
	CC2500_FOCCFG_value,
	CC2500_BSCFG_value,
	CC2500_AGCCTRL2_value,
	CC2500_AGCCTRL1_value,
	CC2500_AGCCTRL0_value,
	CC2500_WOREVT1_value,
	CC2500_WOREVT0_value,
	CC2500_WORCTRL_value,
	CC2500_FREND1_value,
	CC2500_FREND0_value,
	CC2500_FSCAL3_value,
	CC2500_FSCAL2_value,
	CC2500_FSCAL1_value,
	CC2500_FSCAL0_value,
	CC2500_RCCTRL1_value,
	CC2500_RCCTRL0_value,
	CC2500_FSTEST_value,
	CC2500_PTEST_value,
	CC2500_AGCTEST_value,
	CC2500_TEST2_value,
	CC2500_TEST1_value,
	CC2500_TEST0_value
};
const unsigned char CC2500_patable_value[8]=
{
	CC2500_PATABLE0,
	CC2500_PATABLE1,
	CC2500_PATABLE2,
	CC2500_PATABLE3,
	CC2500_PATABLE4,
	CC2500_PATABLE5,
	CC2500_PATABLE6,
	CC2500_PATABLE7
};

volatile unsigned int newPacket = FALSE;
volatile unsigned int	rx_buff0,
								rx_buff1,
								rx_buff2,
								rx_buff3,
								rx_buff4,
								rx_buff5,
								
								tx_buff0,
								tx_buff1,
								tx_buff2,
								tx_buff3,
								tx_buff4,
								tx_buff5;
volatile unsigned int commEnabled=FALSE;
volatile unsigned int commCount=3;
volatile PACKET rxPacket,txPacket;
volatile unsigned int noCommCount;

unsigned char CC2500_status_byte(void)
{
	unsigned char temp;
	CC2500_CSN = 0;
	__delay32(11);
	while( MISO );
	enable_spi();
	temp = SPI_putc(0x00);
	disable_spi();
	CC2500_CSN = 1;
	return temp;
}
//---------------------------------------------------------------
//
//---------------------------------------------------------------
void CC2500_write_register(unsigned char address,unsigned char data)
{
	CC2500_CSN = 0;
	__delay32(11);
	while( MISO );
	enable_spi();
	SPI_putc(address);	 
	SPI_putc(data);
	disable_spi();
	CC2500_CSN=1;
}
//---------------------------------------------------------------
//
//---------------------------------------------------------------
void CC2500_command(unsigned char command)
{
	CC2500_CSN = 0;
	__delay32(11);
	while( MISO );
	enable_spi();
	SPI_putc(command);
	disable_spi();
	CC2500_CSN = 1;
}
//---------------------------------------------------------------
//
//---------------------------------------------------------------
unsigned char CC2500_read_status_register(unsigned char address)
{
	unsigned char data;
	CC2500_CSN = 0;
	__delay32(11);
	while( MISO );
	enable_spi();
	SPI_putc( address | 0xC0 );
	data = SPI_putc( 0x00 );
	disable_spi();
	CC2500_CSN = 1;
	return data;
}
//---------------------------------------------------------------
//
//---------------------------------------------------------------
unsigned char CC2500_read_register(unsigned char address)
{
	unsigned char data;
	CC2500_CSN=0;
	__delay32(11);
	while( MISO );
	enable_spi();
	SPI_putc( address | 0x80 );
	data = SPI_putc( 0x00 );
	disable_spi();
	CC2500_CSN=1;
	return data;
}
//---------------------------------------------------------------
//
//---------------------------------------------------------------
void CC2500_POR(void)
{
	CC2500_CSN=1;
	__delay_us(10);
	SCLK=0;				
	MOSI=0;
	
	CC2500_command(SIDLE);
	
	CC2500_CSN=1;
	__delay_us(10);
	CC2500_CSN=0;
	__delay_us(10);
	CC2500_CSN=1;
	__delay_us(50);
	CC2500_CSN=0;
	
	__delay32(11);
	while( MISO );
	
	enable_spi();
	SPI_putc(SRES);
	disable_spi();
	__delay32(11);
	while( MISO );
	
	MOSI = 0;
	CC2500_CSN=1;
}
//---------------------------------------------------------------
//
//---------------------------------------------------------------
void CC2500_Init_registers(void)
{

	unsigned int add;
	CC2500_CSN = 0;
	__delay32(11);
	while( MISO );
	enable_spi();
	SPI_putc(0x40);	 
	for(add=0; add<0x2F; add++)
	{
		SPI_putc( CC2500_settings[add] );
	}
	disable_spi();
	CC2500_CSN=1;
}
//---------------------------------------------------------------
//
//---------------------------------------------------------------
void CC2500_Init_PA_table(void)
{
	unsigned int add;
	CC2500_CSN=0;
	__delay32(11);
	while( MISO );
	enable_spi();
	SPI_putc( PATABLE & 0x40 );
	for( add=0; add<8; add++ )
	{
		SPI_putc( CC2500_patable_value[add] );
	}
	disable_spi();
	CC2500_CSN=1;
}
//---------------------------------------------------------------
//
//---------------------------------------------------------------
void CC2500_transmit_packet(void)
{
	CC2500_CSN=0;
	__delay32(11);
	while( MISO );
	enable_spi();
	SPI_putc( TXFIFO | 0x40 );
	SPI_putc(FELLOW_ADD);
	SPI_putc(tx_buff0);
	SPI_putc(tx_buff1);
	SPI_putc(tx_buff2);
	SPI_putc(tx_buff3);
	SPI_putc(tx_buff4);
	SPI_putc(tx_buff5);
	disable_spi();
	CC2500_CSN=1;
	CC2500_command( STX );
	/*while(GDO0==0);	//wait till syc word starts to tx  
	while(GDO0==1);	//wait till end of packet ,GD0X_CFG=0x06*/
}
//-----------------------------------------------------------------
// 
//-----------------------------------------------------------------
unsigned int CC2500_receive_packet(void)
{
	unsigned int temp;
	temp = CC2500_read_status_register( RXBYTES );
	if(temp == PACKET_LEN)
	{
		CC2500_CSN = 0;
		__delay32(11);
		while( MISO );
		enable_spi();
		SPI_putc( RXFIFO | 0xC0 );
					  SPI_putc(0x00);	//discard address
		rx_buff0 = SPI_putc(0x00);
		rx_buff1 = SPI_putc(0x00);
		rx_buff2 = SPI_putc(0x00);
		rx_buff3 = SPI_putc(0x00);
		rx_buff4 = SPI_putc(0x00);
		rx_buff5 = SPI_putc(0x00);
		/*for( cnt=0; cnt<PACKET_LEN; cnt++)
		{
			rx_buff[cnt] = SPI_putc( 0x00 );
		}*/
		disable_spi();
		CC2500_CSN=1;
		newPacket = TRUE;	// Should be cleared at rx_buff read time
	}
	return temp;
}

void CC2500_init(void)
{
	CC2500_CSN	= 1;
	MISO		= 0;
	MOSI		= 0;
	SCLK		= 0;
	GDO0		= 0;
	CC2500_CSN_TRIS=0;
	MISO_TRIS		= 1;
	MOSI_TRIS		= 0;
	SCLK_TRIS		= 0;
	GDO0_TRIS		= 1;
	SPI_init();
	disable_spi();
	CC2500_POR();
	CC2500_Init_registers();
	CC2500_Init_PA_table();
	CC2500_command( SRX );
}

void assamble_packet(void)
{
	//if(txPacket==GENERAL_PKT)
	//{
		//tx_buff0 = GENERAL_PKT;
		tx_buff0 = myCube[0];
		tx_buff1 = myCube[1];
		tx_buff2 = myCube[2];
		tx_buff3 = myCube[3];
		tx_buff4 = my.location;
		if(my.delivering)
			tx_buff4 |= 0x80;
		tx_buff5 = (my.obzEntrCnt<<4) | my.heading;
//	}
	/*if(tx_Packet==MY_DATA_REQ)
	{
		tx_buff0 = MY_DATA_PKT;
		tx_buff1 = fellowCube[0];
		tx_buff2 = fellowCube[1];
		tx_buff3 = fellowCube[2];
		tx_buff4 = fellowCube[3];
		tx_buff5 = fellow.smInd;
		tx_buff6 = 0x00;
	}*/
}

void deassamble_packet(void)
{
	fellowCube[0] = rx_buff0;
	fellowCube[1] = rx_buff1;
	fellowCube[2] = rx_buff2;
	fellowCube[3] = rx_buff3;
	if(rx_buff4&0x80)
		fellow.delivering = TRUE;
	else
		fellow.delivering = FALSE;
	fellow.location = rx_buff4&0x7F;
	fellow.heading = rx_buff5 & 0x0F;
	fellow.obzEntrCnt = (rx_buff5>>4)&0x0F;
}
