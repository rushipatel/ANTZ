#ifndef CC2500_H
#define CC2500_H

#ifndef PACKET_LEN
#define PACKET_LEN 255
#warning"Packet Length 255"
#endif
//---------------------------------------------------------------
//CC2500 register address
//---------------------------------------------------------------
#define IOCFG2				0x00
#define IOCFG1       	0x01
#define IOCFG0D      	0x02
#define FIFOTHR      	0x03
#define SYNC1        	0x04
#define SYNC0        	0x05
#define PKTLEN       	0x06
#define PKTCTRL1     	0x07
#define PKTCTRL0     	0x08
#define ADDR         	0x09
#define CHANNR       	0x0A
#define FSCTRL1      	0x0B
#define FSCTRL0      	0x0C
#define FREQ2        	0x0D
#define FREQ1        	0x0E
#define FREQ0        	0x0F
#define MDMCFG4      	0x10
#define MDMCFG3      	0x11
#define MDMCFG2      	0x12
#define MDMCFG1      	0x13
#define MDMCFG0      	0x14
#define DEVIATN      	0x15
#define MCSM2        	0x16
#define MCSM1        	0x17
#define MCSM0        	0x18
#define FOCCFG       	0x19
#define BSCFG        	0x1A
#define AGCCTRL2     	0x1B
#define AGCCTRL1     	0x1C
#define AGCCTRL0     	0x1D
#define WOREVT1      	0x1E
#define WOREVT0      	0x1F
#define WORCTRL      	0x20
#define FREND1       	0x21
#define FREND0       	0x22
#define FSCAL3       	0x23
#define FSCAL2       	0x24
#define FSCAL1       	0x25
#define FSCAL0       	0x26
#define RCCTRL1      	0x27
#define RCCTRL0      	0x28
#define FSTEST       	0x29
#define PTEST        	0x2A
#define AGCTEST      	0x2B
#define TEST2        	0x2C
#define TEST1        	0x2D
#define TEST0        	0x2E
// Status Registers------------
#define PARTNUM      	0x30
#define VERSION      	0x31
#define FREQEST      	0x32
#define LQI          	0x33
#define RSSI         	0x34
#define MARCSTATE    	0x35
#define WORTIME1     	0x36
#define WORTIME0     	0x37
#define PKTSTATUS    	0x38
#define VCO_VC_DAC   	0x39
#define TXBYTES      	0x3A
#define RXBYTES      	0x3B
//------------------------------
#define PATABLE  		0x3E
#define TXFIFO   		0x3F
#define RXFIFO   		0x3F
//---------------------------------------------------------------
//CC2500 register data
//---------------------------------------------------------------
#define CC2500_IOCFG2_value	0x2F
#define CC2500_IOCFG1_value	0x2E
#define CC2500_IOCFG0D_value	0x06
#define CC2500_FIFOTHR_value	0x07
#define CC2500_SYNC1_value		0xD3
#define CC2500_SYNC0_value		0x91
#define CC2500_PKTLEN_value	PACKET_LEN
#define CC2500_PKTCTRL1_value	0x89
#define CC2500_PKTCTRL0_value	0x44
#define CC2500_ADDR_value		MY_ADD
#define CC2500_CHANNR_value	COMM_CHANNEL
#define CC2500_FSCTRL1_value	0x10
#define CC2500_FSCTRL0_value	0x00
#define CC2500_FREQ2_value		0x5D
#define CC2500_FREQ1_value		0x93
#define CC2500_FREQ0_value		0xB1
#define CC2500_MDMCFG4_value	0x0E
#define CC2500_MDMCFG3_value	0x3B
#define CC2500_MDMCFG2_value	0x73
#define CC2500_MDMCFG1_value	0x42
#define CC2500_MDMCFG0_value	0xF8
#define CC2500_DEVIATN_value	0x00
#define CC2500_MCSM2_value		0x07
#define CC2500_MCSM1_value  	0x3F
#define CC2500_MCSM0_value  	0x18
#define CC2500_FOCCFG_value	0x1D
#define CC2500_BSCFG_value		0x1C
#define CC2500_AGCCTRL2_value	0xC7
#define CC2500_AGCCTRL1_value	0x00
#define CC2500_AGCCTRL0_value	0xB0
#define CC2500_WOREVT1_value	0x87
#define CC2500_WOREVT0_value	0x6B
#define CC2500_WORCTRL_value	0xF8
#define CC2500_FREND1_value	0xB6
#define CC2500_FREND0_value	0x10
#define CC2500_FSCAL3_value	0xEA
#define CC2500_FSCAL2_value	0x0A
#define CC2500_FSCAL1_value	0x00
#define CC2500_FSCAL0_value	0x19//0x11
#define CC2500_RCCTRL1_value	0x41
#define CC2500_RCCTRL0_value	0x00
#define CC2500_FSTEST_value	0x59
#define CC2500_PTEST_value		0x7F
#define CC2500_AGCTEST_value	0x3F
#define CC2500_TEST2_value		0x88
#define CC2500_TEST1_value		0x31
#define CC2500_TEST0_value		0x0B
// PATABLE Values ------------------
#define CC2500_PATABLE0			0xFF//0xFE
#define CC2500_PATABLE1			0x00
#define CC2500_PATABLE2			0x00
#define CC2500_PATABLE3			0x00
#define CC2500_PATABLE4			0x00
#define CC2500_PATABLE5			0x00
#define CC2500_PATABLE6			0x00
#define CC2500_PATABLE7			0x00
//---------------------------------------------------------------
//command
//---------------------------------------------------------------
#define SRES     			0x30
#define SFSTXON  			0x31
#define SXOFF    			0x32
#define SCAL     			0x33
#define SRX      			0x34
#define STX      			0x35
#define SIDLE    			0x36
#define SAFC     			0x37
#define SWOR     			0x38
#define SPWD     			0x39
#define SFRX     			0x3A
#define SFTX     			0x3B
#define SWORRST  			0x3C
#define SNOP     			0x3D

//---------------------------------------------------------------
//CC2500 <H&s>98j.F*m
//---------------------------------------------------------------

extern volatile unsigned int	rx_buff0,
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
extern volatile unsigned int newPacket;
extern volatile unsigned int doComm;
extern volatile unsigned int commEnabled;
extern volatile unsigned int commCount;
extern volatile unsigned int noCommCount;

void CC2500_write_register(unsigned char address,unsigned char data);
void CC2500_command(unsigned char command);
unsigned char CC2500_read_status_register(unsigned char address);
unsigned char CC2500_read_register(unsigned char address);
void CC2500_POR(void);
void CC2500_Init_registers(void);
void CC2500_Init_PA_table(void);
void CC2500_transmit_packet(void);
unsigned int CC2500_receive_packet(void);
void CC2500_init(void);
void assamble_packet(void);
void deassamble_packet(void);
unsigned char CC2500_status_byte(void);
//---------------------------------------------------------------
//macros
//---------------------------------------------------------------
#define CC2500_idle_mode()							CC2500_command(SIDLE)
#define CC2500_clear_rx_fifo()					CC2500_command(SFRX)
#define CC2500_clear_tx_fifo()					CC2500_command(SFTX)
#define CC2500_frequency_calibration()			CC2500_command(SCAL)
#define CC2500_sleep_mode()						CC2500_command(SPWD)
#define CC2500_receive_mode()						CC2500_command(SRX)
#define CC2500_transmit_mode()					CC2500_command(STX)

typedef enum{
	GNERAL_PKT,
	MY_DATA_REQ,
	MY_DATA_PKT,
	VISITED_REQ,
	VISITED_PKT
} PACKET;

extern volatile PACKET rxPacket,txPacket;

#endif
