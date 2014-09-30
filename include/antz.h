#ifndef ANTZ_H
#define ANTZ_H

#include <p33FJ64MC804.h>
#define FCY 29480000UL

#define MACHINE_B

#define COMM_CHANNEL 150

#ifdef MACHINE_B
	#define MY_ADD 0xAA
	#define FELLOW_ADD 0x55
#endif

#ifdef MACHINE_A
	#define MY_ADD 0x55
	#define FELLOW_ADD 0xAA
#endif

#define TRUE  1
#define FALSE 0


// sensor thresholds for cube detection
#define LINE_THRESHOLD 400//500
#define RIGHT_THRESHOLD 100
#define LEFT_THRESHOLD  100
#define FRONT_THRESHOLD 75

#define ORTHO_FEC      11360  // where we are when we get an ortho intersection starts 

#define FORWARD_OFFSET	5412//5569
#define REAR_OFFSET		3781

// motors
// Primus physical data
//
//  Wheel diameter: 55.0 mm
//  Mouse diameter: 82.0 mm    
//  mouse is driven through 16*200 = 3200 microsteps per revolution
//  counts per millimeter = 3200 / 55*3.14159 = 18.5 counts/mm
//  counts per degree = 18.5 * 82 * 3.14159/360 = 13.2
//  approximating this to 13 leaves us 8 degrees under in a full turn
//  which goes to illustrate why approximations ar not good
#define COUNTS_PER_CELL   16913L //(15560L)
#define COUNTS_PER_360DEG (10795L)

//  conversion macros
#define COUNTS_PER_CM (COUNTS_PER_CELL / 25) // round to nearest whole number
#define COUNTS_PER_DIAGONAL ((COUNTS_PER_CELL * 362)/256) // approx 1/sqrt(2)
#define COUNTS_PER_DEG (COUNTS_PER_360DEG/360)          

#define TICKS_PER_SECOND 1000
//  speed cm/s to counts/tick. use 256 for 8.8 fixed point
//  speed must be less than 640cm/s and greater than 5cm/s
#define CONV_SPEED(speed) (COUNTS_PER_CELL*(speed)*256/25/TICKS_PER_SECOND)
//  acceleration cm/s/s to counts/tick/tick
//  smallest useful value for acc is 25cm/s/s 
#define CONV_ACC(acc) (COUNTS_PER_CELL*(acc)*256/25/TICKS_PER_SECOND/TICKS_PER_SECOND)

//  degrees/sec to counts/tick
//  overflows are a problem here so we approximate 256/1000 as 10/39
//  smallest angular speed is 0.3deg/sec 
#define CONV_ANG_SPEED(ang_speed) (COUNTS_PER_360DEG*(ang_speed)*10/39/360)
//  angular acceleration
//  smallest increment is about 300 deg/s/s  
#define CONV_ANG_ACC(ang_acc) (COUNTS_PER_360DEG*(ang_acc)*10/39/360/1000)
//  plain distance conversions
//  distances are stored as longs (24.8)
#define CONV_CM(dist) (COUNTS_PER_CELL*(dist)/25)
#define CONV_CELLS(cells) (COUNTS_PER_CELL*256*(cells))
#define CONV_DEG(deg) (COUNTS_PER_360DEG*256*(deg)/360)
//  scale up for diagonals - overflow opportunity here
#define CONV_STRAIGHT_DIAGONAL(dist) ((362*(dist))/256)

#define EXPLORE_SPEED CONV_SPEED(50)     // 0.5m/s
#define STR_MAX_SPEED CONV_SPEED(40)    // 3.0m/s
#define DIA_MAX_SPEED CONV_SPEED(30) 
#define REVERCE_SPEED CONV_SPEED(30)   

#define IP_ACC        CONV_ANG_ACC(1000) // acceleration used for in-place turns
#define DEFAULT_ACC   CONV_ACC(70)      // how fast we accelerate if we forget to set a value
#define ACC_INC       CONV_ACC(50)       // how much to turn up acceleration after each run

#define PICK_DIST	1
#define DEPO_DIST	2
#define SQUARES_2_DIST 3
#define SQUARES_3_DIST 4
#define ORTHO_SENS_DIST 5

#define NORTH 		0
#define NORTHEAST	1
#define EAST  		2
#define EASTSOUTH	3
#define SOUTH 		4
#define SOUTHWEST	5
#define WEST  		6
#define WESTNORTH	7
#define NODIR		8

#define NORTH_WALL  	1
#define EAST_WALL   	2
#define SOUTH_WALL	4
#define WEST_WALL		8
#define VISITED		16
#define BOUNDARY		32
#define OBZ				64
#define ONROUTE		128

#define NOTNORTH_WALL  (255-NORTH_WALL)
#define NOTEAST_WALL   (255-EAST_WALL)
#define NOTSOUTH_WALL  (255-SOUTH_WALL)
#define NOTWEST_WALL   (255-WEST_WALL)
#define ALLWALLS  \
	(NORTH_WALL|EAST_WALL|SOUTH_WALL|WEST_WALL)
#define NOWALL	0xF0

typedef enum
{
	MC_START,
	MC_SCAN,
	MC_GET_M_THR,
	MC_DELIVER,
	MC_DELIVER_FINISH,
}MACHINE_STATE;

extern MACHINE_STATE mcState,mcNextState;




/**********************************************************************/
/* input buttons                                                      */
/* buttons can be on arbitrary pins                                   */
/* note that analog inputs need setting to digital before use         */
/**********************************************************************/
#define BTN_UP_BIT    10
#define BTN_DN_BIT    11
#define BTN_GO_BIT    7
#define BTN_UP_TRIS   TRISB
#define BTN_DN_TRIS   TRISB
#define BTN_GO_TRIS   TRISA
#define BTN_UP_PU	  CNPU2bits.CN16PUE	// To enable Pull-ups
#define BTN_DN_PU	  CNPU1bits.CN15PUE

#define BTN_UP        (PORTBbits.RB10)
#define BTN_DN        (PORTBbits.RB11)
#define BTN_GO        (PORTAbits.RA7)

/* we need to know where they get mapped in the button state variable */
#define B_UP  2
#define B_DN  1
#define B_GO  0

/**********************************************************************/
/* CC2500 & SPI1 pin Configuartion                                    */
/**********************************************************************/
#define CC2500_CSN	PORTAbits.RA2
#define MISO			PORTCbits.RC3
#define MOSI			PORTCbits.RC5
#define SCLK			PORTCbits.RC4
#define GDO0			PORTCbits.RC8
#define CC2500_CSN_TRIS _TRISA2
#define MISO_TRIS		_TRISC3
#define MOSI_TRIS		_TRISC5
#define SCLK_TRIS		_TRISC4
#define GDO0_TRIS		_TRISC8

#define SDO1RPnR	RPOR10bits.RP21R	
#define SCK1RPnR	RPOR10bits.RP20R
#define SDI1_R		19

#define PACKET_LEN	7

/**********************************************************************/
/*	motors                                                             */
/**********************************************************************/
#define MOTOR_FORWARD   	0
#define MOTOR_BACKWARD  	1

#define LEFT_MOTOR_L_TRIS  	TRISBbits.TRISB15
#define LEFT_MOTOR_H_TRIS   	TRISBbits.TRISB14
#define LEFT_MOTOR_L       	LATBbits.LATB15
#define LEFT_MOTOR_H       	LATBbits.LATB14

#define RIGHT_MOTOR_L_TRIS 	TRISBbits.TRISB13
#define RIGHT_MOTOR_H_TRIS		TRISBbits.TRISB12
#define RIGHT_MOTOR_L      	LATBbits.LATB13
#define RIGHT_MOTOR_H      	LATBbits.LATB12

#define FPWM 20000

#define MOTORS_MAX_DC 736	//half of period, used to limit the duty cycle

#define MOTORS_KP 30
#define MOTORS_KD 100
#define MOTORS_KI 0
#define MOTORS_KO 10
#define STR_KP	-15
#define STR_KD	-50
#define STR_KI	0
#define STR_KO	10

#define QEI1_SWAP_AB 0
#define QEI2_SWAP_AB 1

/**********************************************************************/
/*  System Timer                                                      */
/**********************************************************************/
#define SYSTIM_PERIOD     1000
#define SYSTIM_PR_COUNT	 (FCY/SYSTIM_PERIOD)
#define SYSTIM_INTERRUPT  _T5Interrupt        // interrupt handler
#define SYSTIM_TMR        TMR5                // timer register
#define SYSTIM_PR         PR5                 // period register
#define SYSTIM_CON        T5CON               // control register
#define SYSTIM_CONbits    T5CONbits           // control register bits
#define SYSTIM_IF         IFS1bits.T5IF       // interrupt flag
#define SYSTIM_IE         IEC1bits.T5IE       // interrupt enable bit
#define SYSTIM_IP         IPC7bits.T5IP       // interrupt priority
#define SYSTIM_TON        T5CONbits.TON       // timer enable bit

/**********************************************************************/
/*		LED															  */
/**********************************************************************/ 
#define LED_TRIS     TRISA
#define LED_PORT     PORTA
#define LED_LAT      LATA
#define LED_BIT      10
#define LED_SET_OUT  { LED_TRIS &= ~(1<<LED_BIT); }
#define LED_ON       { LED_LAT  |=  (1<<LED_BIT); }
#define LED_OFF      { LED_LAT  &= ~(1<<LED_BIT); }
#define LED_TOGGLE   { LED_LAT  ^=  (1<<LED_BIT); }

/**********************************************************************/
/* Hardware reset type flags                                          */
/**********************************************************************/ 
#define POR             (RCONbits.POR == 1) 
#define BOR             (RCONbits.BOR == 1) 
#define WDTO            (RCONbits.WDTO == 1)
#define SWR             (RCONbits.SWR == 1)
#define EXTR            (RCONbits.EXTR == 1)
#define IOPUWR          (RCONbits.IOPUWR == 1) 
#define TRAPR				(RCONbits.TRAPR == 1)

#define UNKNOWN_RESET     0
#define POWER_ON_RESET    1
#define BROWN_OUT_RESET   2
#define WATCHDOG_RESET    3
#define SOFTWARE_RESET    4
#define EXTERNAL_RESET    5
#define EXCEPTION_RESET   6
#define TRAP_RESET		  7

/**********************************************************************/
/*  Sensor hardware assignents                                        */
/**********************************************************************/
// emitter on time os used in a simple software delay.
// be sure this does not get optimised out!
#define EMITTER_ON_TIME     10			// time to hold emitters on

#define LEFT_IN_TRIS		TRISBbits.TRISB0
#define FRONT_IN_TRIS	TRISAbits.TRISA1
#define RIGHT_IN_TRIS	TRISAbits.TRISA0
#define LINE_1_IN_TRIS	TRISCbits.TRISC1
#define LINE_2_IN_TRIS	TRISCbits.TRISC0
#define LINE_3_IN_TRIS	TRISBbits.TRISB3
#define LINE_4_IN_TRIS	TRISBbits.TRISB2

#define SET_ADPCFG		AD1PCFGL=0x0008

// sensor input ADC channels
#define LEFT_CHAN   2
#define FRONT_CHAN  1
#define RIGHT_CHAN  0
#define LINE_1_CHAN 7
#define LINE_2_CHAN 6
#define LINE_3_CHAN 5
#define LINE_4_CHAN 4

#define LINE_3_OUT_TRIS	TRISAbits.TRISA9
#define LINE_4_OUT_TRIS	TRISAbits.TRISA4
#define RIGHT_OUT_TRIS	TRISBbits.TRISB4
#define LEFT_OUT_TRIS	TRISAbits.TRISA3
#define LINE_1_OUT_TRIS	TRISCbits.TRISC6
#define LINE_2_OUT_TRIS	TRISBbits.TRISB9
#define FRONT_OUT_TRIS	TRISBbits.TRISB8

#define LINE_3_OUT	LATAbits.LATA9
#define LINE_4_OUT	LATAbits.LATA4
#define RIGHT_OUT		LATBbits.LATB4
#define LEFT_OUT		LATAbits.LATA3
#define LINE_1_OUT	LATCbits.LATC6
#define LINE_2_OUT	LATBbits.LATB9
#define FRONT_OUT		LATBbits.LATB8

/**********************************************************************/
/*  Servo							                                        */
/**********************************************************************/

#define FOLD_ARMS		setServoPos(2,172) 	//retracted
#define CLOSE_ARMS	setServoPos(130,65)	//grabbed
#define UNFOLD_ARMS	setServoPos(170,3)	//open



// battery voltage monitor
// raw battery voltage is fed via a potential divider
// to the ADC port, reducing the true voltage by 0.248
// Since the ADC is reading 0-3.3V as 0-1023, we have 
// to convert the ADC reading to a battery voltage
// KBATT is a constant used to multiply up the raw ADC
// reading before dividing by 512 to give an integer 
// representing the battery voltage in tenths of a volt
#define KBATT 66
#define BATTERY_TRIS TRISCbits.TRISC2
#define BATTERY_CHAN 8
#define ADC_SPEED 800000UL   //ADC samples per second speed
/**************************************************************/
/**************************************************************/

// types used by the maze solver
typedef unsigned char CELL_T;
typedef CELL_T CELL;
typedef CELL_T *MAZE;            

// function pointer type for the menu system
typedef void (*ACTION)(void);

  
// parameters used for executing the movements
typedef struct {
	unsigned int startOffset;    // how far we are into the square at the beginning
	unsigned int endOffset;      // how far we get into the finishing square
	unsigned int entrySpeed;     // how fast we need to be at the entry and exit of the turn
	unsigned int exitSpeed;
	unsigned int innerSpeed;
	unsigned int outerSpeed;
	unsigned int turningSteps;   // number of steps actually turning - half each end
	unsigned int entrySteps;
	unsigned int straightSteps;  // a small straight section in the middle to allow things to catch up
	                             // actually the constant radius part of the turn?
} TURNPARAMS;
                                      
typedef struct
{
	volatile unsigned int heading;
	volatile unsigned int location;
	volatile unsigned int goalSquare;
	volatile unsigned int coveredSqr[4];
	volatile unsigned int obzClear;
	volatile unsigned int obzInd;
	volatile unsigned int delivering;
	volatile unsigned int smInd;
	volatile unsigned int obzEntrCnt;
} STATUS;

extern volatile STATUS my,fellow,st_temp;
extern char outBuf[50];
extern unsigned char cmlInd;
extern unsigned int displayEnabled;

void printStatus(STATUS *);
void getMeThere(void);

#endif
