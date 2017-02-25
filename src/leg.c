#include <stdio.h>
#include <math.h>
#include "ax12a.h"
#include "leg.h"

#define SPEED 20.0     // TODO - use this as linear speed rather than motor speed
#define START_X 0.0    // starting (relaxed) x coordinate for any leg
#define START_Y 15.0   // starting (relaxed) y coordinate for any leg
// !! below two values should be calculated from START_Y
#define X_OFFSET 14.0  // offset x coordinates for the 4 "corner" legs
#define Y_OFFSET 8.0   // offset y coordinates for the 4 "corner" legs
// below are measured from the center of the body to the point at which the coxa pivots at the body
#define Y_CORNER_OFFSET 4.7 // cm from center in the forward/backward axis
#define X_CORNER_OFFSET 7.4 // cm from center in the left/right axis
#define Y_CENTER_OFFSET 6.8
// TODO - the two ^above coordinates need set according
// to START_X and START_Y (ZOFFSET would be synonymous
// with start_z

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

// convert degrees to radians
double to_radians(double degrees) {
    return degrees / (180.0 / M_PI);
}

//given the angles of the servo motors, returns the x,y,z coordinates
//which the tip of the leg should be at
int get_position(struct coordinate* coord, struct position* pos)
{
    //get the angles of all three servos
    float angle1 = to_radians(pos->angle1); //angle of the inner most motor
    float angle2 = to_radians(pos->angle2); //angle of the middle motor
    float angle3 = to_radians(pos->angle3); //angle of the outer most motor
    
    //calculate distance from innermost motor to outermost tip
    float L = sqrt(sq(FEMUR) + sq(TIBIA) - 2*FEMUR*TIBIA*cos(angle3));
    
    //calculate angle between FEMUR and L
    double a2 = acos((sq(TIBIA)-sq(L)-sq(FEMUR))/(-2*L*FEMUR));
    double a1 = angle2 - a2;
    
    double L1 = COXA + (L * sin(a1));  // maybe try fabsf(L * sin(a1) - needs testing
    printf("L1: %f\tL: %f\ta1: %f a2: %f\n",L1,L,a1,a2);
    
    double tan_gamma = tan(angle1);
    double x_num = L1 * tan_gamma;
    double x_denom = sqrt(sq(1/cos(angle1))+1);
    
    //calculate final coordiates
    coord->z = ZOFFSET - (L * cos(a1));
    coord->x = L1*sin(angle1);
    coord->y = L1*cos(angle1);
    
    return 1;
}

// Given a leg number and the angles of the servos, calculates the coordinates at which
// the tip of the leg are, relative to where the leg connects to the body
void get_position_relative(int leg_num, struct coordinate* coord, struct position* pos)
{
    // we don't want to modify the struct we were given, but want to pass a
    // modified one to get_position()
    static struct position adjusted_pos;
    adjusted_pos.angle1 = pos->angle1;
    adjusted_pos.angle2 = pos->angle2;
    adjusted_pos.angle3 = pos->angle3;
    
    // adjust angle1 + or - 45 degrees according to its orientation on the body
    switch(leg_num)
    {
        case 0:
            adjusted_pos.angle1 = pos->angle1 + 45;
            break;
        case 2:
            adjusted_pos.angle1 = pos->angle1 - 45;
            break;
        case 3:
            adjusted_pos.angle1 = pos->angle1 + 45;
            break;
        case 5:
            adjusted_pos.angle1 = pos->angle1 - 45;
            break;
    }
    
    // get_position() finds the coordinates from the angles in our diagrams,
    // these values need to be calculated from the default angles of our servos
    adjusted_pos.angle1 -= 150;
    adjusted_pos.angle2 -= 60;
    adjusted_pos.angle3 = 360 - (adjusted_pos.angle3 + 138);
    // printf("get_pos angles: 1:%f 2:%f 3:%f",adjusted_pos.angle1,adjusted_pos.angle2,adjusted_pos.angle3);
    get_position(coord, &adjusted_pos);
    
    // adjust the given x and y coordinates to the centered position of the corner legs
    switch(leg_num)
    {
        case 0:
            coord->x-=X_OFFSET;
            coord->y+=Y_OFFSET;
            break;
        case 2:
            coord->x+=X_OFFSET;
            coord->y+=Y_OFFSET;
            break;
        case 3:
            coord->x-=X_OFFSET;
            coord->y+=Y_OFFSET;
            break;
        case 5:
            coord->x+=X_OFFSET;
            coord->y+=Y_OFFSET;
            break;
    }
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
    z = ZOFFSET - coord->z;
    
    // calculate desired angle of servo 1
    pos->angle1 = get_gamma(x, y);
    double L1 = sqrt(sq(x) + sq(y)); // This is the distance from the tip of the leg to its pivot point at the central body when viewed from top down... can be used together with servo #1 speed in order to calculate the speed of the tip along circumference of circle
    
    // calculate desired angle of servo 2
    double L = sqrt(sq(z) + sq(L1 - COXA));
    double a1 = acos(z/L);
    double a2 = acos((sq(TIBIA) - sq(FEMUR) - sq(L)) / (-2*FEMUR*L));
    
    pos->angle2 = to_degrees(a1 + a2);
    //printf("L1: %f\tL: %f\ta1: %f a2: %f\n",L1,L,a1,a2);
    
    // calculate desired angle of servo 3
    pos->angle3 = to_degrees(acos((sq(L) - sq(TIBIA) - sq(FEMUR)) / (-2*TIBIA*FEMUR)));
    
    //printf("ANGLES:\n1: %f\n2: %f\n3: %f\n", pos->angle1, pos->angle2, pos->angle3);
    
    // !! should L1 also be a member of the position struct and returned for future use?
    return 1;	//!! maybe it would be best to return success code dependent on whether or not the leg can be moved to that position (angles are not NaN)
}

// same as get_angles() above, but the given x, y and z axes
// are parallel to the x, y and z axes of any other leg
int get_angles_relative(int leg_num, struct position* pos, struct coordinate* coord)
{
    static struct coordinate adjusted_coord; //the adjusted coordinate relative to the legs position on the body
    adjusted_coord.x = coord->x;
    adjusted_coord.y = coord->y;
    adjusted_coord.z = coord->z;
    
    switch(leg_num)
    {
        case 0:
            adjusted_coord.x+=X_OFFSET;
            adjusted_coord.y-=Y_OFFSET;
            break;
        case 2:
            adjusted_coord.x-=X_OFFSET;
            adjusted_coord.y-=Y_OFFSET;
            break;
        case 3:
            adjusted_coord.x+=X_OFFSET;
            adjusted_coord.y-=Y_OFFSET;
            break;
        case 5:
            adjusted_coord.x-=X_OFFSET;
            adjusted_coord.y-=Y_OFFSET;
            break;
    }
    
    get_angles(pos, &adjusted_coord);
    
    switch(leg_num)
    {
        case 0:
            pos->angle1 = pos->angle1 - 45;
            break;
        case 2:
            pos->angle1 = pos->angle1 + 45;
            break;
        case 3:
            pos->angle1 = pos->angle1 - 45;
            break;
        case 5:
            pos->angle1 = pos->angle1 + 45;
            break;
    }
    
    pos->angle1 = pos->angle1 + 150;     // get_angles() currently only returns the angles from the inverse kinematics diagrams we found online, these values need adjusted to the default angles for our servos (I should probably just have resolved this in get angles...)
    pos->angle2 = pos->angle2 + 60;
    pos->angle3 = 360 - pos->angle3 - 138;
    
    //printf("ANGLES:\n1: %f\n2: %f\n3: %f\n", pos->angle1, pos->angle2, pos->angle3);
    
    return 1;   //!! maybe it would be best to return success code dependent on whether or not the leg can be moved to that position (angles are not NaN)
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
    }
}

// given angle from center, and radius of circle to trace with tips of legs,
// calculates the x,y, and z coordinates that the tip of leg should be at
// (coordinates are from (0,0) being the center of the body, x axis is horizontal, and y is verticle)
// (theata should be given in degrees)
int get_rotate_location(struct coordinate* coord, double theta, double r)
{
    //!! should z be set here?
    coord->z = 0;
    coord->x = r * cos(to_radians(theta));
    coord->y = r * sin(to_radians(theta));
}

// given leg number, angle from center (calculate from OFFSETS (both internal and external)), and radius of circle to trace with tips
// of legs, calculates the x,y, and z coordinates that the tip of leg should be at
int get_rotate_location_relative(int leg_num, struct coordinate* coord, double theta, double r)
{
    printf("theta = %f\n",theta);
    // adjust theta relative to leg_number
    double theta_offset = 0;
    switch (leg_num)
    {
        case 0:
            theta_offset = 180 - to_degrees(atan((X_CORNER_OFFSET+X_OFFSET)/(Y_CORNER_OFFSET+Y_OFFSET)));
            break;
        case 1:
            theta_offset = 180;
            break;
        case 2:
            theta_offset = 180 + to_degrees(atan((X_CORNER_OFFSET+X_OFFSET)/(Y_CORNER_OFFSET+Y_OFFSET)));
            break;
        case 3:
            theta_offset = 360 - to_degrees(atan((X_CORNER_OFFSET+X_OFFSET)/(Y_CORNER_OFFSET+Y_OFFSET)));
            break;
        case 4:
            theta_offset = 0;
            break;
        case 5:
            theta_offset = 0 + to_degrees(atan((X_CORNER_OFFSET+X_OFFSET)/(Y_CORNER_OFFSET+Y_OFFSET)));
            break;
    }

    theta += theta_offset;
    printf("adjusted_theta = %f\n",theta);

    get_rotate_location(coord, theta, r);
    printf("coord->x = %f\tcoord->y = %f\n", coord->x, coord->y);

    // the coordinates returned from get_rotate_position are relative to x-y coordinate ple from center of body
    // however, legs take a coordinate plane that is rotated 90 degrees, hence x_coord = y and y_coord = x
    double temp_x = coord->y;
    double temp_y = coord->x;

    //adjust coordinate
    switch(leg_num)
    {
        case 0:
            //temp_y is negative
            coord->y = -temp_y - Y_CORNER_OFFSET - Y_OFFSET;
            coord->x = temp_x - X_CORNER_OFFSET - X_OFFSET;
            break;
        case 1:
            //tempy is negative, temp_x may be negative (what we want?) !! OR may need to flip x... +-
            coord->y = -temp_y - Y_CENTER_OFFSET - 17;    // !! ALL these occurances of 17 should be changed to START_Y
            coord->x = temp_x;
            break;
        case 2:
            //coord->y is neg, coord->x is neg
            coord->y = -temp_y - Y_CORNER_OFFSET - Y_OFFSET;
            coord->x = temp_x + X_CORNER_OFFSET + X_OFFSET;
            break;
        case 3:
            coord->y = temp_y - Y_CORNER_OFFSET - Y_OFFSET;
            coord->x = -temp_x - X_CORNER_OFFSET - X_OFFSET;
            break;
        case 4:
            coord->y = temp_y - Y_CENTER_OFFSET - 17; // !! START_Y
            coord->x = -temp_x;
            break;
        case 5:
            coord->y = temp_y - Y_CORNER_OFFSET - Y_OFFSET;
            coord->x = -temp_x + X_CORNER_OFFSET + X_OFFSET;
            break;
    }
    
    printf("adjusted: coord->x = %f\tcoord->y = %f\n", coord->x, coord->y);
    //coord->y += 17; // !! this should be START_Y

    return 1;
}

// moves the tip of a leg to the given coordinate
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
// moves legs based on the angles passed in
int move_leg_angles(int leg_num, double degree1, double degree2, double degree3){
	int* servos = legs[leg_num];
	turnMotor(servos[0], degree1, 5);
        turnMotor(servos[1], degree2, 5);
        turnMotor(servos[2], degree3, 5);
	return 0;
}

// move the tip of a leg to a coordinate relative to its location on the body
int move_leg_relative(int leg_num, struct coordinate* coord)
{
    int* servos = legs[leg_num];
    struct position pos;
    get_angles_relative(leg_num, &pos, coord);
    turnMotor(servos[0], pos.angle1, SPEED);
    turnMotor(servos[1], pos.angle2, SPEED);
    turnMotor(servos[2], pos.angle3, SPEED);
}
