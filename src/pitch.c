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
#define PI 3.14159265358979323846

struct coordinate coord;

int move_leg_angles(int leg_num, double degree1, double degree2, double degree3);

//program that sets the legs back to their default (resting) positions;
int resetAllLegs()
{
	coord.x = START_X;
	coord.y = START_Y;
	coord.z = START_Z;
	int i=0;
	for(i=0;i<6;i++){
		move_leg(i, &coord);
	}

}

void roll(double beta){
	//Input validations
	if(fabsf(beta) > 60.0){
		printf("Input exceeded max slope\n");
		return;
	}
	if(beta == 0.0){
		resetAllLegs();
		return;
	}

	//calculating z offset and x offset
	double theta = 0.5 * (180.0 - beta);
	theta = theta * PI / 180.0;			//convert to radians
	beta = beta * PI / 180.0;				//convert to radians
	double hypotenuse = 2.0 * sin((beta/2.0)) * (BODYWIDTH/2.0);
	double delta_z = cos((PI/2)-theta) * hypotenuse;
	double delta_x = sin((PI/2)-theta) * hypotenuse;
	printf("beta (in rad):%f\n", beta);
	printf("theta:%f; hypotenuse:%f\n", theta, hypotenuse);
	printf("delta z:%f ; delta x:%f\n", delta_z, delta_x);

	//determine what kind of roll
	if(beta > 0){
		printf("Right roll\n");
	}else{
		printf("Left roll\n");
	}

	//Set right set of legs
	coord.x = START_X;
	coord.y = START_Y + delta_x;
	coord.z = START_Z + delta_z;
	move_leg(4, &coord);
	coord.x = START_X + (delta_x * sin(PI/4));		//= 45 degrees in radian
	coord.y = START_Y + (delta_x * sin(PI/4));
	move_leg(3, &coord);
	move_leg(5, &coord);

	//set left set of legs
	coord.x = START_X;
	coord.y = START_Y - delta_x;
	coord.z = START_Z - delta_z;
	move_leg(1, &coord);
	coord.x = START_X - (delta_x * sin(PI/4));		//= 45 degrees in radian
	coord.y = START_Y - (delta_x * sin(PI/4));
	move_leg(0, &coord);
	move_leg(2, &coord);


}


void pitch(double alpha){
		//input validation
		if(fabsf(alpha) > 60.0){
			printf("Input exceeded max slope");
			return;
		}
    if(alpha == 0.0){
			resetAllLegs();
			return;
    }

    //calculating z offset and y offset
    double theta = 0.5 * (180.0 - alpha);
		theta = theta * PI / 180.0; //convert to radians
		alpha= alpha * PI / 180.0;
    double hypotenuse = 2.0 * sin((alpha/2.0)) * (BODYLENGTH/2.0);
    double delta_z = cos((PI/2)-theta) * hypotenuse;
    double delta_y = sin((PI/2)-theta) * hypotenuse;
		printf("alpha in radians:%f\n", alpha);
		printf("theta:%f; hypotenuse:%f\n", theta, hypotenuse);
		printf("delta z:%f ; delta y:%f\n", delta_z, delta_y);

/*  //TODO: Work on pitching from "current" positions instead
    struct leg_status legstat0;
    struct leg_status legstat2;
    struct leg_status legstat3;
    struct leg_status legstat5;
    get_leg_status(0, &legstat0);
    get_leg_status(2, &legstat2;
    get_leg_status(3, &legstat3);
    get_leg_status(5, &legstat5);
*/


		if(alpha < 0){
			printf("Back pitch\n");
		}else{
			printf("Forward pitch\n");
		}

		coord.x = START_X + (delta_y * sin(PI/4));		//= 45 degrees in radian
		coord.y = START_Y + (delta_y * sin(PI/4));
		coord.z = START_Z + delta_z;
		printf("front legs: x:%f ; y:%f ; z:%f\n", coord.x, coord.y, coord.z);
		move_leg(0, &coord);
		move_leg(5, &coord);

		coord.x = START_X - (delta_y * sin(PI/4));
		coord.y = START_Y - (delta_y * sin(PI/4));
		coord.z = START_Z - delta_z;
		printf("back legs: x:%f ; y:%f ; z:%f\n", coord.x, coord.y, coord.z);
		move_leg(2, &coord);
		move_leg(3, &coord);

}

int main(void)
{
    char portName[] = "/dev/ttyUSB0";
    char buffer[20];
    int i, motorID, retval;
    float degree, speed;

    openPort(portName);
		resetAllLegs();

		printf("Select mode:\n\t1. Pitch\n\t2. Roll\n");
		scanf(" %s", buffer);
		int choice = atoi(buffer);

    while(1){
			/* //FOR TESTING ONLY
				struct coordinate coord;
				get_current_leg_position(1,&coord);
				printf("Leg 1 position: x:%f; y:%f; z:%f;\n", coord.x, coord.y, coord.z);
			*/
        printf("Select angle: ");
        scanf(" %s", buffer);
				if(buffer[0] == 'q'){
					break;
				}
        double alpha = atoi(buffer);
				if(choice == 1)     pitch(alpha);
				if(choice == 2)			roll(alpha);
				else break;
    }

    closePort(portName);

}
