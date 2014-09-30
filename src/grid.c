#include "antz.h"
#include "libpic30.h"
#include <stdio.h>
#include "uart.h"
#include "grid.h"
#include "profiler.h"
#include "cc2500.h"
#include "servo.h"
#include "systemtimer.h"
#include "sensors.h"

unsigned char grid[256],map[256];
unsigned char scanMap[17];
const unsigned char SCAN_MAP_A[]=
{29,20,21,22,23,24,33,42,41,40,39,38,47,56,57,58,59};
const unsigned char SCAN_MAP_B[]=
{51,60,59,58,57,56,47,38,39,40,41,42,33,24,23,22,21};
volatile unsigned char myCube[4]={81,81,81,81};
volatile unsigned char fellowCube[4]={81,81,81,81};
unsigned int cubeInd;
unsigned int cubeIndForMcDeliver;

void updateCube(unsigned char cube)
{
	unsigned int i;
	for(i=0; i<4; i++)
	{
		if((myCube[i]==cube)||(fellowCube[i]==cube))
			return;
	}
	myCube[++cubeInd]=cube;
}

unsigned char unDipoCube(void)
{
	int i;
	for(i=0; i<4; i++)
	{
		if(myCube[i]<81)
		{
			if(!isDipoSquare(myCube[i]))
				return myCube[i];
		}
	}
	return FALSE;
}

unsigned int isDipoSquare(unsigned char sqr)
{
	if(sqr==44)
		return TRUE;
	if(sqr==76)
		return TRUE;
	if(sqr==36)
		return TRUE;
	if(sqr==4)
		return TRUE;
	return FALSE;
}

unsigned char nearestNeighbourOfSqr(unsigned char sqr)
{
	unsigned char leastValue = 255,neighbour=0;
	unsigned int i;
	updateGrid();
	floodGrid(my.location);
	for(i=0; i<8; i+=2)
	{
		if(map[nextSquare(sqr,i)]<leastValue)
		{
			sqr = nextSquare(sqr,i);
			leastValue = map[nextSquare(sqr,i)];
		}
	}
	return neighbour;
}

unsigned char nearestNeighbourOfCube(void)
{
	unsigned char leastValue = 255,sqr=0,temp;
	unsigned int i,j;
	updateGrid();
	floodGrid(my.location);
	for(i=0; i<4; i++)
	{
		temp = myCube[i];
		if((temp<81) && (!isDipoSquare(temp)))
		{
			for(j=0; j<8; j+=2)
			{
				if(map[nextSquare(temp,j)]<leastValue)
				{
					sqr = nextSquare(temp,j);
					leastValue = map[nextSquare(temp,j)];
					cubeIndForMcDeliver = i;
				}
			}
		}
	}
	return sqr;
}

unsigned char getDipoSquare(void)
{
	unsigned char leastValue = 255,dipoSqr=0;
	waitDataUpdate();
	updateGrid();
	floodGrid(my.location);
	waitDataUpdate();
	if((!commonCube(44))&&(leastValue>map[44]))
	{
		leastValue = map[44];
		dipoSqr = 44;
	}
	if((!commonCube(76))&&(leastValue>map[76]))
	{
		leastValue = map[76];
		dipoSqr = 76;
	}
	if((!commonCube(36))&&(leastValue>map[36]))
	{
		leastValue = map[36];
		dipoSqr = 36;
	}
	if((!commonCube(4))&&(leastValue>map[4]))
	{
		leastValue = map[4];
		dipoSqr = 4;
	}
	return dipoSqr;
}
unsigned int scanNeighbours(void)
{
	unsigned char temp_sqr;
	unsigned int cubeInd_temp;
	cubeSensorsOn();
	waitToTick();
	waitToTick();
	cubeSensorsOff();
	cubeInd_temp = cubeInd;
	temp_sqr = nextSquare(my.location,(my.heading-2)&0x07);
	if(!commonSquare(temp_sqr))
	{
		grid[temp_sqr] |= VISITED;
		if(leftCube)
			updateCube(temp_sqr);
	}
	temp_sqr = nextSquare(my.location,(my.heading+2)&0x07);
	if(!commonSquare(temp_sqr))
	{
		grid[temp_sqr] |= VISITED;
		if(rightCube)
			updateCube(temp_sqr);
	}
	temp_sqr = nextSquare(my.location,my.heading&0x07);
	if(!commonSquare(temp_sqr))
	{
		grid[temp_sqr] |= VISITED;
		if(frontCube)
			updateCube(temp_sqr);
	}
	/*temp_sqr = nextSquare(my.location,(my.heading-2)&0x07);
	if(!commonSquare(temp_sqr)&&!(grid[temp_sqr]&BOUNDARY))
	{
		grid[temp_sqr] |= VISITED;
		if(leftCube)
			updateCube(temp_sqr);
	}
	temp_sqr = nextSquare(my.location,(my.heading+2)&0x07);
	if(!commonSquare(temp_sqr)&&!(grid[temp_sqr]&BOUNDARY))
	{
		grid[temp_sqr] |= VISITED;
		if(rightCube)
			updateCube(temp_sqr);
	}
	temp_sqr = nextSquare(my.location,my.heading&0x07);
	if(!commonSquare(temp_sqr)&&!(grid[temp_sqr]&BOUNDARY))
	{
		grid[temp_sqr] |= VISITED;
		if(frontCube)
			updateCube(temp_sqr);
	}*/
	if(cubeInd_temp==cubeInd)
		return FALSE;
	return TRUE;
}

unsigned char nextNeighbour(unsigned char sqr)//49uS
{
	unsigned char dist,nextSqr,leastSqr;
	unsigned char temp_grid,dir;
	temp_grid=grid[sqr];
	dist=map[sqr];
	if(!dist)
		return sqr;
	leastSqr=sqr;
	for(dir=0; dir<8; dir+=2)		//also change in nearestCube()
	{
		nextSqr = nextSquare(sqr,dir);
		if(!commonSquare(nextSqr))
		{
			if( temp_grid & BOUNDARY )
			{
				if( !(temp_grid & wallOnDirection(dir)) )
				{
					if(map[nextSqr]<dist)
					{
						dist = map[nextSqr];
						leastSqr = nextSqr;
					}	
				}
			}
			else
			{
				if(map[nextSqr]<dist)
				{
					dist = map[nextSqr];
					leastSqr = nextSqr;
				}
			}
		}
	}
	return leastSqr;
}

unsigned int commonSquare(unsigned char sqr)
{
	int i;
	for(i=0; i<4; i++)
	{
		if(fellow.coveredSqr[i]<81)
		{
			if(fellow.coveredSqr[i]==sqr)
				return TRUE;
		}	
	}
	return FALSE;
}
unsigned int commonCube(unsigned char sqr)
{
	int i;
	for(i=0; i<4; i++)
	{
		if(fellowCube[i]==sqr)
			return TRUE;
	}
	return FALSE;
}
unsigned char wallOnDirection(unsigned char dir)
{
	switch(dir)
	{
		case 0:
			return NORTH_WALL;
		case 1:
			return (NORTH_WALL|EAST_WALL);
		case 2:
			return EAST_WALL;
		case 3:
			return (EAST_WALL|SOUTH_WALL);
		case 4:
			return SOUTH_WALL;
		case 5:
			return (SOUTH_WALL|WEST_WALL);
		case 6:
			return WEST_WALL;
		case 7:
			return (WEST_WALL|NORTH_WALL);
		default:
			return 0x00;
	}
}


void deliverCube(void)
{
	unsigned char dir,diff;
	while(prState!=PR_FINISHED);
	dir = nextDirection(my.location,my.goalSquare);
	diff = ((dir+4)&0x07) - (my.heading&0x07);
	commandList[cmlInd++] = getTurnCmd( diff );
	commandList[cmlInd++] = CMD_STRAIGHT | DEPO_DIST;
	commandList[cmlInd++] = CMD_STOP;
	getNextMove();
	__delay_ms(10);
	while(prState!=PR_FINISHED);
	UNFOLD_ARMS;
	__delay_ms(100);
	__delay_ms(100);
	commandList[cmlInd++] = CMD_STRAIGHT | CMD_FORWARD | DEPO_DIST;
	commandList[cmlInd++] = CMD_STOP;
	getNextMove();
	__delay_ms(10);
	while(relativeDistance<(CONV_CM(12)*2));
	my.delivering = FALSE;
	FOLD_ARMS;
	my.heading = (dir+4) | NODIR;
	my.heading &= 0x0F;
}

void grabCube(unsigned char dir)
{
	unsigned char diff,nextSqr;
	
	nextSqr = nextSquare(my.location,dir);
	waitDataUpdate();
	if( commonSquare(nextSqr) )
		return;
	
	diff = ((dir+4)&0x07) - (my.heading&0x07);
	if(diff&0x01)
		return;
	my.heading = dir;
	while(prState!=PR_FINISHED);
	UNFOLD_ARMS;
	commandList[cmlInd++] = getTurnCmd( diff );
	commandList[cmlInd++] = CMD_STRAIGHT;
	commandList[cmlInd++] = CMD_STOP;
	getNextMove();
	while(prState!=PR_FINISHED);
	CLOSE_ARMS;
	__delay_ms(100);
	__delay_ms(100);
	my.location = nextSqr;
	my.heading = (my.heading + 4) | NODIR;
	my.heading &= 0x0F;
	my.delivering = TRUE;
}

void updateGrid(void)//121uS
{
	unsigned char i,j;
	for(i=0; i<81; i++)
	{
		grid[i] &= NOWALL;
	}
	if( !inOBZ(my.location) && !my.obzClear && !my.delivering )
	{
		for(i=0; i<9; i++)	// four vertical x 9 obz grid
		{
			grid[i]		|= ALLWALLS;
			grid[i+9]	|= ALLWALLS;
			grid[i+63]	|= ALLWALLS;
			grid[i+72]	|= ALLWALLS;
		}
		for(i=18; i<63; i+=9)	// four horizontal x 5 obz grid
		{
			grid[i]		|= ALLWALLS;
			grid[i+1]	|= ALLWALLS;
			grid[i+7]	|= ALLWALLS;
			grid[i+8]	|= ALLWALLS;
		}
		j=20;
		for(i=20; i<25; i++)	// central zone boundary
		{
			grid[i]	   |= WEST_WALL;	//Western boundary in cz 
			grid[i+36]  |= EAST_WALL;	//Eastern boundary in cz
			grid[j]	   |= SOUTH_WALL;	//Southern boundary in cz
			grid[j+4]   |= NORTH_WALL;	//Northen boundary in cz
			j += 9; 
		}
		grid[4]  &= NOTEAST_WALL;
		grid[13] &= NOTEAST_WALL & NOTWEST_WALL;
		grid[22] &= NOTWEST_WALL;
		
		grid[36] &= NOTNORTH_WALL;
		grid[37] &= NOTNORTH_WALL & NOTSOUTH_WALL;
		grid[38] &= NOTSOUTH_WALL;
		
		grid[44] &= NOTSOUTH_WALL;
		grid[43] &= NOTSOUTH_WALL & NOTNORTH_WALL;
		grid[42] &= NOTNORTH_WALL;
		
		grid[76] &= NOTWEST_WALL;
		grid[67] &= NOTWEST_WALL & NOTEAST_WALL;
		grid[58] &= NOTEAST_WALL;
	}
	
	j=0;
	for(i=0; i<9; i++)
	{
		grid[i]	  |= WEST_WALL;	//Western boundary 
		grid[i+72] |= EAST_WALL;	//Eastern boundary
		grid[j]	  |= SOUTH_WALL;	//Southern boundary
		grid[j+8]  |= NORTH_WALL;	//Northen boundary
		j += 9;
	}
	
	for(i=0; i<4; i++)
	{
		if(my.goalSquare != myCube[i])
			updateWall(myCube[i],ALLWALLS);
		updateWall(fellowCube[i],ALLWALLS);
		updateWall(fellow.coveredSqr[i],ALLWALLS);
	}
}


void updateWall(unsigned char sqr, unsigned char wall)
{
	unsigned char temp;
	if(wall&NORTH_WALL)
	{
		grid[sqr] |= NORTH_WALL;
		grid[sqr+1] |= SOUTH_WALL;
	}
	if(wall&EAST_WALL)
	{
		grid[sqr] |= EAST_WALL;
		grid[sqr+9] |= WEST_WALL;
	}
	if(wall&SOUTH_WALL)
	{
		grid[sqr] |= SOUTH_WALL;
		temp = sqr+255;
		grid[temp] |= NORTH_WALL;
	}
	if(wall&WEST_WALL)
	{
		grid[sqr] |= WEST_WALL;
		temp = sqr+247;
		grid[temp] |= EAST_WALL;
	}
}

unsigned int inOBZ(unsigned char sqr)
{
	/*int i;
	for(i=0; i<9; i++)
	{
		if( sqr==i || sqr==i+9 || sqr==i+63 || sqr==i+72 )
			return TRUE;
	}
	for(i=18; i<63; i+=9)
	{
		if( sqr==i || sqr==i+1 || sqr==i+7 || sqr==i+8 )
			return TRUE;
	}*/
	
	if(sqr<81)
	{
		if(grid[sqr]&OBZ)
			return TRUE;
	}
	return FALSE;
}

unsigned char OBZ_clear(void)
{
	if(inOBZ(fellow.coveredSqr[0]))
		return FALSE;
	if(inOBZ(fellow.coveredSqr[1]))
		return FALSE;
	if(inOBZ(fellow.coveredSqr[2]))
		return FALSE;
	if(inOBZ(fellow.coveredSqr[3]))
		return FALSE;
	if(my.obzEntrCnt>fellow.obzEntrCnt)
		return FALSE;
	return TRUE;
} 

void getFellowCoveredSqrs(void)//6uS
{
	unsigned char tempDir,tempLoc;
	
	tempLoc = fellow.location;
	tempDir = fellow.heading;
	#ifdef MACHINE_A
		fellow.coveredSqr[0] = tempLoc;
		fellow.coveredSqr[1] = 0xFF;
		fellow.coveredSqr[2] = 0xFF;
		fellow.coveredSqr[3] = 0xFF;
	#endif
	
	if(!(tempDir&0x08))		// if no direction (stoped or desiding)
	{
		if(tempDir&0x01)	// if diagonal move
		{
			fellow.coveredSqr[1] = nextSquare(tempLoc,tempDir);
			tempDir++;		// add 1
			tempDir &= 0x07;
			fellow.coveredSqr[2] = nextSquare(tempLoc,tempDir);
			tempDir += 254;	// subtract by 2
			tempDir &= 0x07;
			fellow.coveredSqr[3] = nextSquare(tempLoc,tempDir);
		}
		else							// orthogonal move
			fellow.coveredSqr[1] = nextSquare(tempLoc,tempDir);
	}
}

void gridInit(void)//206uS
{
	unsigned char i,j;
	
	i=0;
	do{
		grid[i--]=255;
	}while(i);
	
	for(i=0; i<81; i++)
		grid[i] = 0;
	
	j=0;
	for(i=0; i<9; i++)
	{
		grid[i]	   |= WEST_WALL | VISITED | BOUNDARY;	//Western boundary 
		grid[i+72]  |= EAST_WALL | VISITED | BOUNDARY;	//Eastern boundary
		grid[j]	  	|= SOUTH_WALL | VISITED | BOUNDARY;	//Southern boundary
		grid[j+8]   |= NORTH_WALL | VISITED | BOUNDARY;	//Northen boundary
		j += 9;
	}
	for(i=0; i<9; i++)	// four vertical x 9 obz grid
	{
		grid[i]		|= ALLWALLS | OBZ;
		grid[i+9]	|= ALLWALLS | OBZ;
		grid[i+63]	|= ALLWALLS | OBZ;
		grid[i+72]	|= ALLWALLS | OBZ;
	}
	for(i=18; i<63; i+=9)	// four horizontal x 5 obz grid
	{
		grid[i]		|= ALLWALLS | OBZ;
		grid[i+1]	|= ALLWALLS | OBZ;
		grid[i+7]	|= ALLWALLS | OBZ;
		grid[i+8]	|= ALLWALLS | OBZ;
	}
	grid[20] |= VISITED;	//Machine A start squre
	grid[60] |= VISITED;	//MAchine B start squre
	grid[10] |= VISITED;	//Corner squre of comman grid
	grid[16] |= VISITED;	//Corner squre of comman grid
	grid[70] |= VISITED;	//Corner squre of comman grid
	grid[64] |= VISITED;	//Corner squre of comman grid
}

unsigned char floodGrid(unsigned char goal)//2260uS
{
  unsigned char i,j;
  unsigned char now,next;
  unsigned char passes;
  unsigned char cellwalls;    // the wall data for a given cell
  unsigned char changed;

  i = 0;
  do
  {
    map[i--] = 255;
  } while (i);

  map[goal]=0;
  passes = 0;
  now = 0;
  next = now+1;
  do
  {
    changed = 0;
    i = 0;
    do
    {
      if (map[i]==now)
      {
        cellwalls=grid[i];
        if ((cellwalls & NORTH_WALL) == 0)
        {
          j = i+1;
          if (map[j] == 255){ map[j] = next; changed=1;}
        }
        if ((cellwalls & EAST_WALL) == 0)
        {
          j = i + 9;
          if (map[j] == 255){ map[j] = next; changed=1;}
        }
        if ((cellwalls & SOUTH_WALL) == 0)
        {
          j = i + 255;
          if (map[j] == 255){ map[j] = next; changed=1;}
        }
        if ((cellwalls & WEST_WALL) == 0)
        {
          j = i + 247;
          if (map[j] == 255){ map[j] = next; changed=1;}
        }
      }
      i--;
    } while(i);
    now  = now+1;
    next = now+1;
    passes++;
  } while(changed);

  return passes;
}

unsigned char nextDirection(unsigned char currentSqr, unsigned char nextSqr)
{
	signed char diff;
	diff = nextSqr - currentSqr;
	switch(diff)
	{
		case 1 :
			return 0;
		case 10 :
			return 1;
		case 9 :
			return 2;
		case 8 :
			return 3;
		case -1 :
			return 4;
		case -10 :
			return 5;
		case -9 :
			return 6;
		case -8 :
			return 7;
		default :
			return NODIR;
	}
}
unsigned char nextSquare(unsigned char sqr, unsigned char dir)
{
	switch(dir)
	{
		case 0 :
			return (sqr+1);
		case 1 :
			return (sqr+10);
		case 2 :
			return (sqr+9);
		case 3 :
			return (sqr+8);
		case 4 :
			return (sqr+255);
		case 5 :
			return (sqr+246);
		case 6 :
			return (sqr+247);
		case 7 :
			return (sqr+248);
		default  :
			return sqr;
	}
}

unsigned char getTurnCmd(unsigned char error)
{
	error &= 0x07;
	switch(error)
	{
		case 0:
			return CMD_COMMAND ;
		case 1:
			return (CMD_INPLACE | CMD_45 | CMD_RIGHT)  ;
		case 2:
			return (CMD_INPLACE | CMD_90 | CMD_RIGHT)  ;
		case 3:
			return (CMD_INPLACE | CMD_135 | CMD_RIGHT) ;
		case 4:
			return (CMD_INPLACE | CMD_180) ;
		case 5:
			return (CMD_INPLACE | CMD_135) ;
		case 6:
			return (CMD_INPLACE | CMD_90) ;
		case 7:
			return (CMD_INPLACE | CMD_45)  ;
		default:
			return CMD_COMMAND ;
	}
}

void printGrid(void)//248uS
{
	unsigned int add,i,j,k;
	if(displayEnabled)
	{
		putsUART("\r\n");
		for(i=0; i<48; i++)
			putUART('-');
		putsUART("\r\n");
		add=8;
		for(i=0; i<9; i++)
		{
			for(j=0; j<9; j++)
			{
				sprintf(outBuf," | %2x",grid[add]);
				putsUART(outBuf);
				add += 9;
			}
			add -= 82;
			putsUART(" | \r\n");
			for(k=0; k<48; k++)
				putUART('-');
			putsUART("\r\n");
		}
	}
}

void printMap(void)//292uS
{
	unsigned int add,i,j,k;
	if(displayEnabled)
	{
		putsUART("\r\n");
		for(i=0; i<57; i++)
			putUART('-');
		putsUART("\r\n");
		add=8;
		for(i=0; i<9; i++)
		{
			for(j=0; j<9; j++)
			{
				sprintf(outBuf," | %3u",map[add]);
				putsUART(outBuf);
				add += 9;
			}
			add -= 82;
			putsUART(" | \r\n");
			for(k=0; k<57; k++)
				putUART('-');
			putsUART("\r\n");
		}
	}
}

void printRoute(void)
{
	unsigned int add,i,j,k;
	if(displayEnabled)
	{
		putsUART("\r\n");
		for(i=0; i<48; i++)
			putUART('-');
		putsUART("\r\n");
		add=8;
		for(i=0; i<9; i++)
		{
			for(j=0; j<9; j++)
			{
				if(grid[add]&ONROUTE)
					putsUART(" | XX");
				else
				{
					sprintf(outBuf," | %2u",add);
					putsUART(outBuf);
				}	
				add += 9;
			}
			add -= 82;
			putsUART(" | \r\n");
			for(k=0; k<48; k++)
				putUART('-');
			putsUART("\r\n");
		}
	}
}
