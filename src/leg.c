#include <stdio.h>
#include <math.h>
#include "ax12a.h"
#include "leg.h"

#define SPEED 20

//     FRONT
//
//    0     3
//     \   /
//      \-/
//  1---| |---4
//      /-\
//     /   \
//    2     5
//
//     REAR
//
// these are the id's of each motor for each leg (legs are numbered as above)
int legs[6][3] = {{2,15,16},{14,18,19},{8,9,10},{17,3,4},{11,6,7},{5,12,13}};

// square function
double sq(double x)
{
    return x*x;
}

// convert radians to degrees
double to_degrees(double radians) {
    return radians * (180.0 / M_PI);
}

// Given x, y, and z coordinates, returns a position struct containing
// the three angles that the servos need to be at for a single leg
// Equations and variables are based off the diagrams found here :
// https://oscarliang.com/inverse-kinematics-and-trigonometry-basics/
int get_angles(struct position* pos, float x, float y, float z, int leg_num)
{    
    z = ZOFFSET - z; //!!
    // adjust offsets according to leg placement on body
    /*switch(leg_num)
	{
	    case 3:
	    case 4:
	    case 5:
	}*/
    // calculate desired angle of servo 1
    pos->angle1 = to_degrees(atan(x/y));
    double L1 = sqrt(sq(x) + sq(y)); // !! This is the distance from the tip of the leg to its pivot point at the central body when viewed from top down... can be used together with servo #1 speed (hopefully in degrees/sec??) in order to calculate the speed of the tip along circumference of circle (and we can modify the servo speed in order to make this a constant speed in the forward direction??)
    
    // calculate desired angle of servo 2
    double L = sqrt(sq(z) + sq(L1 - COXA));
    double a1 = acos(z/L);
    double a2 = acos((sq(TIBIA) - sq(FEMUR) - sq(L)) / (-2*FEMUR*L));
    pos->angle2 = to_degrees(a1 + a2);
    
    // calculate desired angle of servo 3
    pos->angle3 = to_degrees(acos((sq(L) - sq(TIBIA) - sq(FEMUR)) / (-2*TIBIA*FEMUR)));

    // adjust offsets according to leg placement on body
    switch(leg_num)
	{
	    case 0:
		pos->angle1 = - (pos->angle1 - 45);
		break;
	    case 1:
		pos->angle1 = -pos->angle1;
		break;
	    case 2:
		pos->angle1 = - (pos->angle1 - 45);
		break;
	    case 3:
		pos->angle1 = (pos->angle1 - 45);
		break;
	    case 4:
		// already accounted for
		break;
	    case 5:
		pos->angle1 = (pos->angle1 + 45);
		break;
	}
    printf("ANGLES:\n1: %f\n2: %f\n3: %f\n", pos->angle1, pos->angle2, pos->angle3);
    
    // !! should L1 also be a member of the position struct and returned for future use?
    return 1;	//!! maybe it would be best to return success code dependent on whether or not the leg can be moved to that position (angles are not NAN)
}

int move_leg(int leg_num, float x, float y, float z)
{
    int* servos = legs[leg_num];
    struct position pos;

    get_angles(&pos, x, y, z, leg_num);

    if(isnan(pos.angle1) || isnan(pos.angle2) || isnan(pos.angle3))
    {
        printf("That is not a valid position that can be reached by the tip of the leg\n");
    }
    else
    {
        printf("Moving to position (%f, %f, %f)\n", x, y, z);
        
        // the offests for [this leg] are hard coded here (might need to change for other legs?)
        //  -> for the opposite side, subtract value from 360
        //  -> for legs closer to the front or rear, adjust angle#1 accordingly

	float degree1, degree2, degree3;
	degree1 = pos.angle1 + 150;
	degree2 = pos.angle2 + 60;
        degree3 = 360 - pos.angle3 - 138;
        printf("DEGREES:1: %f\n2: %f\n3: %f\n", degree1, degree2, degree3);

        turnMotor(servos[0], degree1, SPEED);
        turnMotor(servos[1], degree2, SPEED);
        turnMotor(servos[2], degree3, SPEED);
    }
}

// servo1 desired position = angle1 + 150
// servo2 desired position = angle2 + 60
// servo2 desired position = (360-angle3) - 138;  //assuming 42 is the straight position for this servo and 138 = 180-42

/*
int main(int argc, char** argv)
{
    struct position pos = get_angles(0, 20, 0);
    printf("\ngamma : %f\nalpha : %f\nbeta : %f\n\n", pos.angle1, pos.angle2, pos.angle3);
    
}
*/
