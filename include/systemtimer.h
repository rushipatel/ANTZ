#ifndef TIMERCOUNTERS_H
#define TIMERCOUNTERS_H

#include "antz.h"

extern volatile unsigned long  tickCount;
extern volatile unsigned int millisecondCount;
extern volatile unsigned int maxTimer;
extern volatile unsigned int dataUpdated;
extern volatile unsigned int doComm;

void systemTimerInit(void); 
void startSystemTimer(void); 
void stopSystemTimer(void);
void delayMs(unsigned int);
int waitDataUpdate(void);
void waitToTick(void);
void waitBeforTransmit(void);

void __attribute__((interrupt, auto_psv)) SYSTIM_INTERRUPT(void);

#endif

