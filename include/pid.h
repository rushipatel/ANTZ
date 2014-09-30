#ifndef PID_H
#define PID_H

#ifndef PID_MAX_OUTPUT
#define PID_MAX_OUTPUT 32767L
#endif

#ifndef PID_MIN_OUTPUT
#define PID_MIN_OUTPUT -32767L
#endif

typedef struct
{
    volatile long setValue;		// as 24.8, In this application
    volatile long currentValue;	// Current position
    volatile int  Kp;
    volatile int  Kd;
    volatile int  Ki;
    volatile int  Ko;
    volatile long prevErr;
    volatile long Ierror;
}PID_STRUCT;

int doPID (PID_STRUCT * p);

#endif
