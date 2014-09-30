#ifndef MOTORS_H
#define MOTORS_H
#include "pid.h"

/*
 * motor speed is a signed 8.8 quantity representing the number of 
 * motor pulses per millisecond. Thus, for primus, speeds are potentially
 * +/- 127*0.054*1000 m/s = +/- 6.8m/s
 * In practice, half that would be exceptional.
 *
 * acceleration is also an 8.8 number although only the lower 8 bits
 * are used. The minimum acceleration is about 1/256 * 0.054 *1000000
 * which is 0.21m/s/s. even a relatively high acceleration of 5m/s/s
 * would only require an acceleration value of 24
 *
 */
 
extern volatile signed int leftCount,rightCount;
extern PID_STRUCT left_PID_param, right_PID_param;

void motorsInit(void);
void motorsOn(void);
void motorsOff(void);
void motorsUpdateSpeed(void);
void motorsRightSetDutyCycle(int dutyCycle);
void motorsLeftSetDutyCycle(int dutyCycle);
void readCounters(void);

#endif
