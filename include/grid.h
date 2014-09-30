#ifndef GRID_SR_H
#define GRID_SR_H

extern unsigned char grid[256],map[256];
extern unsigned char scanMap[17];
extern volatile unsigned char myCube[4],fellowCube[4];
extern unsigned int cubeInd;
extern const unsigned char SCAN_MAP_A[];
extern const unsigned char SCAN_MAP_B[];
extern unsigned int cubeIndForMcDeliver;


void getFellowCoveredSqrs(void);
void mazeInit(void);
unsigned char floodGrid(unsigned char goal);
unsigned char nextDirection(unsigned char currentSqr, unsigned char nextSqr);
unsigned char nextSquare(unsigned char sqr, unsigned char dir);
unsigned char getTurnCmd(unsigned char error);
unsigned int inOBZ(unsigned char sqr);
void updateWall(unsigned char sqr, unsigned char wall);
void grabCube(unsigned char);
void deliverCube(void);
void gridInit(void);
void updateGrid(void);
unsigned char nextNeighbour(unsigned char);
void printGrid(void);
void printMap(void);
void printRoute(void);
unsigned char wallOnDirection(unsigned char);
unsigned int commonSquare(unsigned char);
unsigned int commonCube(unsigned char);
unsigned char OBZ_clear(void);
unsigned int scanNeighbours(void);
unsigned char getDipoSquare(void);
unsigned int isDipoSquare(unsigned char sqr);
unsigned char unDipoCube(void);
void updateCube(unsigned char cube);
unsigned char nearestNeighbourOfCube(void);
//unsigned char nearestNeighbourOfSqr(unsigned char sqr);

#endif
