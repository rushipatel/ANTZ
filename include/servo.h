#ifndef	SERVO_H
#define SERVO_H

#define SERVO_MAX 58165//7370
#define SERVO_MIN 58165-3685//3685
#define DIFF_DC (SERVO_MAX-SERVO_MIN)

#define SERVO_LEFT_DCR  OC1RS
#define SERVO_RIGHT_DCR OC2RS

void servoInit(void);
void setServoPos(unsigned long,unsigned long);

#endif
