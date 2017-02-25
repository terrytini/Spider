#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "ax12a.h"
#include "leg.h"
#include "input.h"

#define BODYLENGTH 15.0
#define BODYWIDTH 9.5
#define DEFAULTANGLE1 150.0
#define DEFAULTANGLE2 180.0
#define DEFAULTANGLE3 150.0


int move_leg_angles(int leg_num, double degree1, double degree2, double degree3);

int resetAllLegs()
{
	int i=0;
	for(i=0;i<6;i++){
		move_leg_angles(i, DEFAULTANGLE1, DEFAULTANGLE2, DEFAULTANGLE3);
	}

}

void pitch(double alpha){
    if(alpha == 0.0){
	resetAllLegs();
	return;
    }

    //calculating z offset and x offset
    double theta = 0.5 * (180 - alpha);
    double hypotenuse = 2 * sin(alpha/2) * BODYLENGTH;
    double delta_z = cos(90-theta) * hypotenuse;
    double deltz_x = sin(90-theta) * hypotenuse;

    struct leg_status legstat0;
    struct leg_status legstat1;
    struct leg_status legstat4;
    struct leg_status legstat5;

    get_leg_status(0, &legstat0);
    get_leg_status(1, &legstat1);
    get_leg_status(4, &legstat4);
    get_leg_status(5, &legstat5);


    double a = fabsf(alpha);    

    //TODO: PROPERLY CALCULATE CHANGES
    double frontchange2 =-1 * a;    
    double frontchange3 = -1 * a;
    double backchange2 = a;
    double backchange3 = -1 * a;

    printf("backchange: %f; %f\n",backchange2, backchange3);
    //if(backchange2 > 15) backchange2 = -15;

    //back pitch
    int front1 = 0;
    int front2 = 5;
    int back1 = 2;
    int back2 = 3;

    if(alpha < 0){
	front1 = 3;
        front2 = 2;
        back1 = 0;
        back2 = 5;
    }

    //want to move the front to the correct location
    move_leg_angles(front1,DEFAULTANGLE1, DEFAULTANGLE2+frontchange2, DEFAULTANGLE3+frontchange3);
    move_leg_angles(front2,DEFAULTANGLE1, DEFAULTANGLE2+frontchange2, DEFAULTANGLE3+frontchange3);

    //move the back to the correct location
    move_leg_angles(back1, DEFAULTANGLE1, DEFAULTANGLE2+backchange2, DEFAULTANGLE3+backchange3);
    move_leg_angles(back2, DEFAULTANGLE1, DEFAULTANGLE2+backchange2, DEFAULTANGLE3+backchange3);
   
}

int main(void)
{
    char portName[] = "/dev/ttyUSB0";
    char buffer[20];
    int i, motorID, retval;
    float degree, speed;
    
    openPort(portName);

    while(1){
        printf("Select angle: ");
        scanf(" %s", buffer);
        double alpha = atoi(buffer);

        pitch(alpha);
    }

    closePort(portName);

}


