#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "ax12a.h"
#include "leg.h"

int turnRobotCCW();
int resetAllLegs();
int resetAllLegsBack();
int rotateGroupOneCCW();
int rotateLegCCW(int motorID1, int motorID2, int motorID3);
int wave(int motorID1, int motorID2, int motorID3, int numwaves);
int testMotor();
int resetLeg(int motorID1, int motorID2, int motorID3);
int resetLegBack(int motorID1, int motorID2, int motorID3);

#define DEFAULT_BAUDRATE 1000000
#define USB_LATENCY 4000
int terminal_fd;

int main(int argc, char *argv[])
{
    if(argc < 2){ 
	return -1;
    }

    char portName[] = "/dev/ttyUSB0";
    terminal_fd = openPort(portName);

	if(terminal_fd == 0) return 0;

	struct timespec tim, tim2;
	tim.tv_sec = 0;
	tim.tv_nsec = 500000000L;

	resetAllLegs();
	sleep(2);

	if(strcmp(argv[1] , "wave" )== 0){
		wave(8,9,10,6);
		resetLeg(8,9,10);
	}
	else if(strcmp(argv[1] , "turn" )== 0){
		turnRobotCCW();
		turnRobotCCW();	
		turnRobotCCW();	
		turnRobotCCW();	
		resetAllLegs();	
	}
	else if(strcmp(argv[1], "turn1" )==0){
		rotateGroupOneCCW();
		rotateGroupOneCCW();
		rotateGroupOneCCW();
		rotateGroupOneCCW();
		resetAllLegs();
		
	}
	else{
		resetAllLegs();
	}

    close(terminal_fd);

    return 0;
}
int turnRobotCCW(){
	//group 1
	rotateLegCCW(2,15,16);
	rotateLegCCW(11,6,7);
	rotateLegCCW(8,9,10);

	//group 2
	rotateLegCCW(5,12,13);
	rotateLegCCW(14,18,19);
	rotateLegCCW(17,3,4);

	resetAllLegsBack();
}
int resetAllLegs(){
	//reset
	resetLeg(2,15,16);	//group 1
	resetLeg(5,12,13);
	resetLeg(14,18,19);
	resetLeg(11,6,7);	//group 1
	resetLeg(8,9,10);	//group 1
	resetLeg(17,3,4);	
}
int resetAllLegsBack(){
	resetLegBack(2,15,16);	//group 1
	resetLegBack(5,12,13);
	resetLegBack(14,18,19);
	resetLegBack(11,6,7);	//group 1
	resetLegBack(8,9,10);	//group 1
	resetLegBack(17,3,4);
}
//-------------------------------------------------------------------------
int rotateGroupOneCCW(){
	struct timespec tim, tim2;
	tim.tv_sec = 0;
	tim.tv_nsec = 400000000L;

	//leg 1
	turnMotor(19, 125, 12);
	turnMotor(18, 220, 12);
	turnMotor(14, 105, 20);
	//leg 2
	turnMotor(13, 125, 12);
	turnMotor(12, 220, 12);
	turnMotor(5, 105, 20);
	//leg 3
	turnMotor(17, 125, 12);
	turnMotor(3, 220, 12);
	turnMotor(4, 105, 20);

	nanosleep(&tim, &tim2);
	//leg 1
	turnMotor(18, 200, 12);
	turnMotor(19, 150, 12);
	//leg 2
	turnMotor(12, 200, 12);
	turnMotor(13, 150, 12);
	//leg 3
	turnMotor(3, 200, 12);
	turnMotor(4, 150, 12);

	nanosleep(&tim, &tim2);
	resetAllLegsBack();
	tim.tv_sec = 1;
	tim.tv_nsec = 0L;
	nanosleep(&tim, &tim2);
}
//-------------------------------------------------------------------------
int rotateLegCCW(int motorID1, int motorID2, int motorID3){
	struct timespec tim, tim2;
	tim.tv_sec = 0;
	tim.tv_nsec = 400000000L;

	turnMotor(motorID3, 125, 12);
	turnMotor(motorID2, 220, 12);
	turnMotor(motorID1, 105, 20);
	nanosleep(&tim, &tim2);
	turnMotor(motorID2, 200, 12);
	turnMotor(motorID3, 150, 12);
	
}
//-------------------------------------------------------------------------
int wave(int motorID1, int motorID2, int motorID3, int numwaves){
	struct timespec tim, tim2;
	tim.tv_sec = 0;
	tim.tv_nsec = 500000000L;

	turnMotor(motorID2, 230, 20);
	nanosleep(&tim, &tim2);
	
	tim.tv_nsec = 300000000L;

	int i;
	for(i=0; i<numwaves;i++){
		turnMotor(motorID3, 30, 20);
		float position = 0;
		float speed = 0;
		getPresentPositionSpeed(terminal_fd, motorID3, &position, &speed);
		printf("\nMoving Up: Position: %f|| Speed: %f", position, speed);
		nanosleep(&tim, &tim2);
		turnMotor(motorID3, 60, 15);
		getPresentPositionSpeed(terminal_fd, motorID3, &position, &speed);
		printf("\nMoving Down: Position: %f|| Speed: %f", position, speed);
		nanosleep(&tim, &tim2);
	} 
	
	
	
}
//-------------------------------------------------------------------------
int resetLeg(int motorID1, int motorID2, int motorID3)
{
	turnMotor(motorID1, 150, 10);
	turnMotor(motorID2, 200, 10);
	turnMotor(motorID3, 150, 10);

}
int resetLegBack(int motorID1, int motorID2, int motorID3)
{
	turnMotor(motorID1, 195, 10);
	turnMotor(motorID2, 200, 10);
	turnMotor(motorID3, 150, 10);

}
//-------------------------------------------------------------------------
int testMotor()
{
	char buffer[20];
    int i, motorID, retval,speed;
    float degree;

	while(1)
	{
		printf("Enter a motor ID number (0 - 253 or q to quit): ");
		scanf(" %s", buffer);

		if(buffer[0] == 'q')
		{
			break;
		}

		motorID = atoi(buffer);

		printf("Enter a goal position (0 - 300 or q to quit): ");
		scanf(" %s", buffer);

		if(buffer[0] == 'q')
		{
			break;
		}

		degree = atof(buffer);

		printf("Enter speed: ");
		scanf(" %s", buffer);

		if(buffer[0] == 'q')
		{
			break;
		}

		speed = atoi(buffer);

		retval = turnMotor(motorID, degree, speed);

		if(retval != 0)
		{
			printf("Error: %i\n", retval);
		}
		else
		{
			printf("Success!!!\n");
		}
	}
}

