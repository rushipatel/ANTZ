#ifndef PROFILER_H
#define PROFILER_H

#include "pid.h"

/// commands are single bytes containing the type of move and the 
/// appropriate distance or angle.
/// 76543210
/// ||||||||  
/// |||||||\\_____ for turns: 00 =  45deg
/// |||||||                   01 =  90deg
/// |||||||                   10 = 135deg
/// |||||||                   11 = 180deg
/// |||||||\______ for turns:  0 = left
/// ||||||||                   1 = right 
/// ||||||||
/// ||||||||
/// ||||||||
/// ||| \\\\\_____ straights use 5 bits for distance
/// || \__________ for a turn  0 = straight move/entry
/// ||                         1 = diagonal move/entry
///  \\___________ move is    00 = integrated turn
///                           01 = inplace turn
///                           10 = straight
///                           11 = command
///
#define CMD_COMMAND    0xC0
#define CMD_STRAIGHT   0x80
#define CMD_TURN_TYPE  0x60
#define CMD_INPLACE    0x40
#define CMD_DIAGONAL   0x20
#define CMD_RIGHT      0x04
#define CMD_DISTANCE   0x0F
#define CMD_STOP       0xC1
#define CMD_END        0xC2
#define CMD_ANGLE      0x03
#define CMD_45         0x00
#define CMD_90         0x01
#define CMD_135        0x02
#define CMD_180        0x03
#define CMD_FORWARD    0x10

typedef unsigned char COMMAND;
extern COMMAND currentMove, nextMove;
extern COMMAND commandList[256];
extern unsigned char cmdIndex;
extern COMMAND * pCommand;
extern long turnoverPos,currentPos,finalPos;
extern int turnoverSpeed;
extern long relativeDistance;
extern int desiredAcc, desiredDec, currentSpeed,diffSpeed;
extern unsigned char breaksOn;
extern PID_STRUCT steer_PID_param;
extern long distance[];
extern int speed;

typedef union {
  unsigned int entire;
  struct{
    char fractional;        // C30 allocates the low byte first
    char integral;
  } parts;
} SPEED;
    
typedef enum { PR_INIT, 
               PR_STR_ACC,
               PR_STR_BRK,
               PR_STR_DEC,
               PR_TRN_IN,
               PR_TRN_CONST,
               PR_TRN_OUT,
               PR_IPT_ACC,
               PR_IPT_DEC,
               PR_FINISHED 
} PROFILE_STATE;
extern PROFILE_STATE prState;

typedef struct  {
  int avSpeed;
  int diffSpeed;
  unsigned int turnDist;
  unsigned int leadIn;
  unsigned int leadOut;
}  PARAMS ;

typedef struct
{
	int desiredAcc;
	int desiredDec;
	int orthoMaxSpeed;
	int diaMaxSpeed;
	int ipAcc;
} SPEED_PARAM;
                
                
void getNextMove(void);
void profilerInit(void);
void doProfiler(void);
int profileFinished(void);
void doOrthoSteering(void);
                
#endif
