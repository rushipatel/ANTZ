#ifndef SENSORS_H
#define SENSORS_H

/***********************************************
 *  
 *  ANTZ sensors for the cube, line and the battery
 *
 ***********************************************/
// raw sensor readings
extern volatile unsigned int frontSensor;
extern volatile unsigned int leftSensor;
extern volatile unsigned int rightSensor;
extern volatile unsigned int lineSensor[4];

extern volatile int headingError;
extern volatile int offsetError;

extern volatile unsigned int batteryVoltage;

// booleans holding presence/absence of cube
extern volatile unsigned char leftCube;
extern volatile unsigned char rightCube;
extern volatile unsigned char frontCube;
extern volatile unsigned char intersection;
extern volatile unsigned char oldIntersection;

void sensorsOn(void);
void cubeSensorsOn(void);
void cubeSensorsOff(void);
void sensorsOff(void);

void sensorsInit(void);
void readLineSensors(void);
void readCubeSensors(void);
void readAllSensors(void);
int readSensor(int sensor);
void printSensors(void);

unsigned int readBatteryVoltage(void);


#endif

 
