#include <stdio.h>
#include <math.h>
#include "ax12a.h"
#include "leg.h"

#define SPEED 20
#define START_X 0    //starting (relaxed) x coordinate for any leg
#define START_Y 20   //starting (relaxed) y coordinate for any leg

//     FRONT
//
//    0     5
//     \   /
//      \^/
//  1---| |---4
//      /-\
//     /   \
//    2     3
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

//given the angles of the servors motors, returns the x,y,z coordinates
//which the tip of the leg should be at
int get_positions(struct coordinate* coord, struct position* pos ){
    //get the angles of all three servos
	float angle1 = pos->angle1;	//angle of the inner most motor
	float angle2 = pos->angle2;	//angle of the middle motor
	float angle3 = pos->angle3;	//angle of the outer most motor
    //calculate distance from middle motor to outermost tip
	float c = sqrt(FEMUR*FEMUR + TIBIA*TIBIA + 2*FEMUR*TIBIA*cos(angle3));
    //calculate angle between FEMUR and c
	float angle4 = asin(TIBIA * (angle3/c));
    //calculate offsets
	float delta_z = ZOFFSET - (sin(angle4-angle2)*c);
	float delta_y = COXA + (cos(angle4-angle2)*c);
    //calculate final coordiates
	coord->z = delta_z;
	coord->x = delta_y * sin(angle1);
	coord->y = delta_y * sin(angle1);
    return 1;
}

//given x and y, returns the angle of the first servo motor (gamma) 
float get_gamma(float x, float y)
{
    return to_degrees(atan(x/y));
}

// Given x, y, and z coordinates, returns in a position struct
// the three angles that the servos need to be at for a single leg
// Equations and variables are based off the diagrams found here :
// https://oscarliang.com/inverse-kinematics-and-trigonometry-basics/
int get_angles(struct position* pos, struct coordinate* coord)
{
    float x,y,z;
    x = coord->x;
    y = coord->y;
    z = ZOFFSET - coord->z; //!!
    
    // calculate desired angle of servo 1
    pos->angle1 = get_gamma(x, y);
    double L1 = sqrt(sq(x) + sq(y)); // !! This is the distance from the tip of the leg to its pivot point at the central body when viewed from top down... can be used together with servo #1 speed in order to calculate the speed of the tip along circumference of circle (and we can modify the servo speed in order to make this a constant speed in the forward direction??)
    
    // calculate desired angle of servo 2
    double L = sqrt(sq(z) + sq(L1 - COXA));
    double a1 = acos(z/L);
    double a2 = acos((sq(TIBIA) - sq(FEMUR) - sq(L)) / (-2*FEMUR*L));
    pos->angle2 = to_degrees(a1 + a2);
    
    // calculate desired angle of servo 3
    pos->angle3 = to_degrees(acos((sq(L) - sq(TIBIA) - sq(FEMUR)) / (-2*TIBIA*FEMUR)));
    
    //printf("ANGLES:\n1: %f\n2: %f\n3: %f\n", pos->angle1, pos->angle2, pos->angle3);
    
    // !! should L1 also be a member of the position struct and returned for future use?
    return 1;	//!! maybe it would be best to return success code dependent on whether or not the leg can be moved to that position (angles are not NaN)
}

// fills a motor_status struct with position and speed of motor
void get_motor_status(int id, struct motor_status* motor_stat)
{
    getPresentPositionSpeed(id, &motor_stat->position, &motor_stat->speed);
}

// fills a leg_status structure with the status of each servo
void get_leg_status(int leg_num, struct leg_status* leg_stat)
{
    for (int i = 0; i < 3; i++)
    {
        get_motor_status(legs[leg_num][i], &(leg_stat->motors[i]));
        
        /*switch(i)
        {
            case 0:
                leg_stat->motors[i] = ;
                break;
            case 1:
                leg_stat->motors[i] = ;
                break;
            case 2:
                leg_stat->motors[i] = ;
                break;
        }*/
    }
}

// moves a leg to the given coordinate
int move_leg(int leg_num, struct coordinate* coord)
{
    int* servos = legs[leg_num];
    struct position pos;
    get_angles(&pos, coord);
    
    if(isnan(pos.angle1) || isnan(pos.angle2) || isnan(pos.angle3))
    {
        printf("That is not a valid position that can be reached by the tip of the leg\n");
    }
    else
    {
        //printf("Moving to position (%f, %f, %f)\n", x, y, z);
        
        // the offests for [this leg] are hard coded here (might need to change for other legs?)
        //  -> for the opposite side, subtract value from 360
        //  -> for legs closer to the front or rear, adjust angle#1 accordingly
        
        float degree1, degree2, degree3;
        degree1 = pos.angle1 + 150;
        degree2 = pos.angle2 + 60;
        degree3 = 360 - pos.angle3 - 138;
        //printf("DEGREES:1: %f\n2: %f\n3: %f\n", degree1, degree2, degree3);
        
        turnMotor(servos[0], degree1, SPEED);
        turnMotor(servos[1], degree2, SPEED);
        turnMotor(servos[2], degree3, SPEED);
    }
}

// move a leg relative to its location on the body
int move_leg_relative(int leg_num, struct coordinate* coord, float i)
{
    if (leg_num == 0)
    {
        coord->x = START_X-i/sqrt(2);
        coord->y = START_Y-i/sqrt(2);
    }
    else if (leg_num == 2)
    {
        coord->x = START_X-i/sqrt(2);
        coord->y = START_Y+i/sqrt(2);
    }
    else if (leg_num == 3)
    {
        coord->x = START_X-i/sqrt(2);
        coord->y = START_Y-i/sqrt(2);
    }
    else if (leg_num == 5)
    {
        coord->x = START_X-i/sqrt(2);
        coord->y = START_Y+i/sqrt(2);
    }
    else
    {
        coord->x = START_X + i;
    }
    move_leg(leg_num, coord);
}
