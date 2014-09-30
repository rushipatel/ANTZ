#include <math.h>
#include <limits.h>
#include <stdio.h>
#include "antz.h"
#include "profiler.h"
#include "motors.h"
#include "sensors.h"
#include "uart.h"
#include "pid.h"


PID_STRUCT steer_PID_param;
///positions are all stored as 24.8 fixed point values
long currentPos, finalPos;
///speeds are stored as 8.8 fixed point values
int turnoverSpeed;
int diffSpeed, maxSpeed, finalSpeed, currentSpeed  ;
/// Accelerations are stored as 8.8 although only
/// the lower 8 bits are used. Convenient in a 16 bit processor
int desiredAcc,desiredDec,currentAcc;
// boolean flag
unsigned char decelerating;
PROFILE_STATE prState;
COMMAND lastMove,currentMove, nextMove;
COMMAND commandList[256]; // if ever we need more, we are in trouble
//COMMAND * pCommand;
unsigned char cmdIndex;

int remainingSquares;
/// need to keep track of how far we are through a cell. This is used
/// in forward error correction and to decide when it is safe or
/// appropriate to look at the sensors for steering.
/// unlike other distances, a 16 bit integer for convenience
long relativeDistance;
//int relativeDistance;
unsigned char breaksOn=FALSE;
int speed = 1;
/////////////////////////////////////////////////////////////////////////////


  
void doProfiler(void);
void getNextMove(void);
int  getRequiredDec(void);
void initInPlace(void);
void initStraight(void);
void initIntegrated(void);
void profileInPlace(void);
void profileStraight(void);
void profileIntegrated(void);
void updateMotors(void);

SPEED_PARAM speedParam[] = 
{
	//acc, dec, orthoSpeed, diaSpeed, ipAcc
	{CONV_ACC(70),CONV_ACC(100),CONV_SPEED(40),CONV_SPEED(40),CONV_ANG_ACC(1000)},
	{CONV_ACC(90),CONV_ACC(135),CONV_SPEED(60),CONV_SPEED(60),CONV_ANG_ACC(1250)},
	{CONV_ACC(110),CONV_ACC(165),CONV_SPEED(80),CONV_SPEED(80),CONV_ANG_ACC(1500)},
	{CONV_ACC(120),CONV_ACC(180),CONV_SPEED(90),CONV_SPEED(90),CONV_ANG_ACC(1750)},
	{CONV_ACC(130),CONV_ACC(190),CONV_SPEED(95),CONV_SPEED(95),CONV_ANG_ACC(2000)}
};
  
PARAMS params[] = {
  // avSpeed, diffSpeed, turnDist, leadIn, leadOut
  {2560,1280,2000,1000,1000}, // 0x00 = integrated straight entry  45 deg 
  {2560,1280,2000,1000,1000}, // 0x01 = integrated straight entry  90 deg 
  {2560,1280,2000,1000,1000}, // 0x02 = integrated straight entry 135 deg 
  {2560,1280,2000,1000,1000}, // 0x03 = integrated straight entry 180 deg 
  {2560,1280,2000,1000,1000}, // 0x04 = integrated diagonal entry  45 deg 
  {2560,1280,2000,1000,1000}, // 0x05 = integrated diagonal entry  90 deg 
  {2560,1280,2000,1000,1000}, // 0x06 = integrated diagonal entry 135 deg 
  {2560,1280,2000,1000,1000}, // 0x07 = integrated diagonal entry 180 deg 
  {0,0, 1269,0,0},            // 0x08 = in place  45 deg 
  {0,0, 2538,0,0},       	   // 0x09 = in place  90 deg 
  {0,0, 3995,0,0},            // 0x0A = in place 135 deg 
  {0,0, 5075,0,0},            // 0x0B = in place 180 deg 
  {0,0,0,0,0},                // 0x0C = all other moves
};

long distance[]= {
	COUNTS_PER_CELL*256,		//0 One Square
	CONV_CM(18)*256,			//1 Reverse picking
	CONV_CM(16)*256,			//2 Reverse dipositing
	COUNTS_PER_CELL*256*2,	//3 2 squares
	COUNTS_PER_CELL*256*3,	//4 3 Squares
	CONV_CM(16),				//5 ortho sensing distance
	COUNTS_PER_CELL*256*8   //6 Calibration Distance
};

void profilerInit(void){
  desiredAcc = speedParam[speed].desiredAcc;
  desiredDec = speedParam[speed].desiredDec;
  prState = PR_FINISHED;
  currentSpeed = 0;
  diffSpeed = 0;
  commandList[0] = CMD_STOP;
  commandList[1] = CMD_STOP;
  cmdIndex = 0;
  relativeDistance = 0;
  steer_PID_param.setValue = 384L*256L;	//3*256/2
  steer_PID_param.currentValue = 384;
  steer_PID_param.Kp	 = STR_KP;
  steer_PID_param.Kd	 = STR_KD;
  steer_PID_param.Ki	 = STR_KI;
  steer_PID_param.Ko	 = STR_KO;
  steer_PID_param.prevErr = 0;
  steer_PID_param.Ierror = 0;
}  
  
  

void getNextMove(void){
  if (cmdIndex == 0){
    lastMove = CMD_STOP;
  } else {
    lastMove = commandList[cmdIndex-1];
  }
  currentMove = commandList[cmdIndex++];
  nextMove = commandList[cmdIndex];
  if (currentMove == CMD_STOP){
    prState = PR_FINISHED;
    diffSpeed = 0;
    currentSpeed = 0;
  } else {
    prState = PR_INIT;
  }  
}

// This function checks for end of current move
int profileFinished(void)
{
  return prState == PR_FINISHED;
}
  
/*
 Look for trailing edges in the side walls to perform
 forward error correction while travelling along
 orthogonal straights. Both walls are used if available.
 Care must be taken to ensure that we are in a sensible
 area to be looking for trailing edges and that we find 
 genuine edges rather than odd dips in the sensor reading
 The value FEC_OFFSET is a first guess and needs to be adjusted for
 each mouse. 
*/
void doOrthoFEC(void){
  int posError;
	if(relativeDistance>COUNTS_PER_CELL){
	    if ((oldIntersection == 0) && (intersection == 1)){
	    posError = ORTHO_FEC - relativeDistance/2;  // about 230-73-15mm
	    relativeDistance = ORTHO_FEC*2;
	    currentPos += (long)posError * 256;
	  }
 	} 
  oldIntersection = intersection;
}    

/*
 The steering control for orthogonal runs uses the diagonal sensors to
 avoid getting too close to the side walls. Care is taken to avoid errors 
 caused by being too close to a wall in front of the mouse since the
 diagonal sensors get a large return from them. At such times, it might be 
 sensible to use the side sensors.
 Rather than perform simple collision avoidance a 'proper' control system
 should be implemented that attempts to calculate the mouse heading and
 offset error and uses that information to drive a rotational controller.
*/
void doOrthoSteering(void)
{
	unsigned int temp,index, i;
	signed int s_l, s_r, s, error;
	
	diffSpeed = 0;
	temp = 0;
	for(i=0; i<4; i++)
	{
		if( temp<lineSensor[i] )
		{
			temp = lineSensor[i];
			index = i;
		}
	}
	if( index == 0 )
	{
		s_l = 0;
		s	 = lineSensor[0];
		s_r = lineSensor[1];
	}
	else if(index == 3)
	{
		s_l = lineSensor[2];
		s	 = lineSensor[3];
		s_r = 0;
	}
	else
	{
		s_l = lineSensor[index-1];
		s	 = lineSensor[index];
		s_r = lineSensor[index+1];
	}

	error = 128 * ( s_r - s_l);
	error /= ( (2*s) - s_l - s_r );
	
	error += (index*256);
	steer_PID_param.currentValue = error;

	diffSpeed = doPID( &steer_PID_param );
}

void doDiagonalSteering(void)
{
	
}

void doProfiler(void)
{
  COMMAND command;
  if (prState == PR_FINISHED){
    return;
  }  
  relativeDistance += leftCount + rightCount;         // update cell count here!!!!!!!!!!!!!!!!!!!!!
  if (relativeDistance > 2*COUNTS_PER_CELL){
    relativeDistance -= 2*COUNTS_PER_CELL;
  }
  command = currentMove  & CMD_COMMAND;               // mask out the command part
  if (command == CMD_STRAIGHT && (currentMove&CMD_FORWARD)){   // decide on the kind of steering
    if (currentMove & CMD_DIAGONAL){
      // calculate steering correction for diagonal runs
      // and forward error correction
    } else {
    		if( (currentMove&CMD_DISTANCE)==0 )
      		doOrthoFEC();
      doOrthoSteering();
    }
  }
  // see if we are ready to start a new command profile
  if (prState == PR_INIT) {
    if (currentMove == CMD_STOP){
      prState = PR_FINISHED;
      return;
    }
    if (command == CMD_STRAIGHT){ // initialise a straight
      initStraight();
    }  
    if (command == CMD_INPLACE){  // initialise an inplace turn
      initInPlace();
    }
    if (command == CMD_COMMAND){
    	getNextMove();
    } 
  }
  // now execute the appropriate command handler
  if (command == CMD_STRAIGHT){
    profileStraight();
  }
  if (command == CMD_INPLACE){
    profileInPlace();
  }
  
}
  
    
/**
  * Get the deceleration required
  * Deceleration = (curSpeed*curSpeed - endSpeed*endSpeed)/(2*distToDec);
  * Note that speed and acceleration are both fixed pt represented.
  * When the speeds are squared, we get a 16.16 fixed point number.
  * which is then divided by a 24.8 fixed point number. Conveniently, this
  * leaves us an answer neatly scaled to the 8.8 value needed for acceleration
  * the returned value is always positive
  * this is only ever called after we know there is some distance left to travel
  * so there is no need to ensure distance is positive or non-zero
  * There is no division by two as distance travelled is the sum of the left and 
  * right wheel distances
  */
int getRequiredDec(void){
  int dec;
  long distance = finalPos - currentPos;
  if (currentSpeed <= finalSpeed){
    
  }
  dec = ((long)currentSpeed*currentSpeed - (long)finalSpeed*finalSpeed)/distance;
  return (dec)/2 ; 
}
  
void initStraight(void){
  int idx;
  PARAMS * nextParams;
  COMMAND tmpMove;
  decelerating = FALSE;
  tmpMove = nextMove;
  if ((nextMove & CMD_COMMAND) == CMD_STRAIGHT)
    tmpMove = CMD_STOP;
  if (tmpMove == CMD_STOP){
    idx = 12;
  } else {
    idx = (tmpMove & CMD_ANGLE) + ((tmpMove & CMD_TURN_TYPE) >> 3);
  }  
  nextParams = & params[idx]; 
  diffSpeed = 0;
  
  if (currentMove & CMD_DIAGONAL){
    maxSpeed = speedParam[speed].diaMaxSpeed;
    finalPos += distance[currentMove & CMD_DISTANCE] ;
    relativeDistance = 0;
  } 
  else {
	 if(currentMove & CMD_FORWARD)
    	maxSpeed = speedParam[speed].orthoMaxSpeed;
    else
    	maxSpeed = REVERCE_SPEED;
    finalPos += distance[currentMove & CMD_DISTANCE] ;
    relativeDistance = 0;
  }
  
  currentPos = 256000L;        // prevents unexpected moves in a DC mouse
  finalPos +=  256000L;        // while holding position stationary
  currentAcc = desiredAcc;
  finalSpeed = nextParams->avSpeed;
  prState = PR_STR_ACC;
}



void profileStraight(void)
{
  if (currentPos >= (finalPos - 20 * 256)){ // we are happy to get withing 1mm
    finalPos = 0;
    getNextMove();
    return;
  }
	if(breaksOn)
		prState = PR_STR_BRK;
	else
		prState = PR_STR_ACC;
		
  if (prState == PR_STR_BRK){
    if (currentMove & CMD_FORWARD){
      currentSpeed -= desiredDec;
      if (currentSpeed <= 0)
        currentSpeed = 0;
    }
    else{
      currentSpeed += desiredAcc;
      if(currentSpeed >= 0)
        currentSpeed = 0;
    }
  }
  
  if (prState == PR_STR_ACC){
    if (currentMove & CMD_FORWARD){
      currentSpeed += desiredAcc;
      if ( currentSpeed >= maxSpeed){
        currentSpeed = maxSpeed;
      }
      currentAcc = getRequiredDec();
      if (currentAcc >= desiredDec){
        decelerating = TRUE;
        prState = PR_STR_DEC;
      } 
      else
        currentAcc = desiredAcc;
    }
    else{
      currentSpeed -= desiredDec;
      if ( currentSpeed <= -maxSpeed){
        currentSpeed = -maxSpeed;
      }
      currentAcc = getRequiredDec();
      if (currentAcc >= desiredAcc){
        decelerating = TRUE;
        prState = PR_STR_DEC;
      } 
      else
        currentAcc = desiredDec;
    }
  }
  if (prState == PR_STR_DEC){
    currentAcc = getRequiredDec();
    if (currentMove & CMD_FORWARD){
      currentSpeed -= currentAcc;
      if (currentSpeed <= finalSpeed) 
        currentSpeed = finalSpeed;
    }
    else{
      currentSpeed += currentAcc;
      if (currentSpeed >= finalSpeed) 
        currentSpeed = finalSpeed;
    }  
  }
  currentPos += ((leftCount <<8) + (rightCount <<8)) /2;
  // in a DC mouse we would call the PID controller to drive
  // the motors with a torque appropriate to the current positional 
  // error. For a stepper mouse we can only adjust the speed
  motorsUpdateSpeed();
}    
  

/*
 In-place turns are very simple and have two phases.
 the mouse spends the fiorst half accelerating and the 
 second half decelerating. It should end up with both wheels
 stopped and the mouse in the centre of the cell.
*/
void initInPlace(void){
  int idx;
  decelerating = FALSE;
  currentPos = 0;
  currentSpeed = 0;
  if (currentMove & CMD_RIGHT){
    currentAcc = IP_ACC;
  } else {
    currentAcc = -IP_ACC;
  }
  diffSpeed = 0;
  idx = 8 + (currentMove & CMD_ANGLE);
  finalPos = (long)params[idx].turnDist * 256;  // this is just half the turn
  prState = PR_IPT_ACC;
}
  
void profileInPlace(void){
  if (prState == PR_IPT_ACC){         // the first, accelerating phase
    if (currentPos >= finalPos){
      decelerating = TRUE;
      prState = PR_IPT_DEC;
    } else {
      diffSpeed += currentAcc;
      currentPos += ((leftCount <<8) + (rightCount <<8)) /2;
      motorsUpdateSpeed();
    }
  }
  if (prState == PR_IPT_DEC){         // the second, decelerating phase
    if (diffSpeed == 0){
      finalPos = 0;
      relativeDistance = (COUNTS_PER_CELL); // set up for next move
      getNextMove();
    } else {
      diffSpeed -= currentAcc;
      currentPos += ((leftCount <<8) + (rightCount <<8)) /2;
      motorsUpdateSpeed();
    }  
  }
}

void initIntegrated(void)
{
	
}
