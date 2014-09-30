
#ifndef DELAY_H
#define DELAY_H


void delay( unsigned int delay_count);
void delay_us( unsigned int delayUs_count);
void delay_ms (unsigned int milliseconds);

#define delay_us_cnt		(Fcy / 1000000)
#define delay_10us_cnt		(Fcy / 100000)
#define delay_100uS_cnt  	(Fcy / 10000)
#define delay_1mS_Cnt	    (Fcy / 1000)
#define delay_10ms_cnt		(Fcy / 100)

#endif
