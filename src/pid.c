#include "antz.h"
#include <math.h>
#include "pid.h"
#include <stdio.h>
#include "uart.h"

int doPID(PID_STRUCT * p)
{
	long error, output;

	error = (p->setValue/256) - ( (p->currentValue));
	output  = p->Kp * error;
	output += p->Kd * ( error - p->prevErr );
	output += p->Ki * p->Ierror;
	output  = output/p->Ko;
	p->prevErr = error;
	if ( output > PID_MAX_OUTPUT )
		output = PID_MAX_OUTPUT;
	else if (output < PID_MIN_OUTPUT)
		output = PID_MIN_OUTPUT;
	else
		p->Ierror += error;
	return ( (int)output );
}
