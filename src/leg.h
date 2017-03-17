#ifndef _LEGS_H_
#define _LEGS_H_

#define COXA 5.2
#define FEMUR 8.3
#define TIBIA 17
#define ZOFFSET 11.5 //11.5 //8.5
#define START_X 0.0    // starting (relaxed) x coordinate for any leg
#define START_Y 17.0   // starting (relaxed) y coordinate for any leg
#define START_Z 0.0    // starting (relaxed) z coordinate for any leg
#define STEP_Z 4.0     // step height for the tip of the leg

// !! below two values should be calculated from START_Y
#define X_OFFSET 14.0  // offset x coordinates for the 4 "corner" legs
#define Y_OFFSET 8.0   // offset y coordinates for the 4 "corner" legs
//#define X_OFFSET (sqrt((START_Y*START_Y)/2.0))  // offset x coordinates for the 4 "corner" legs
//#define Y_OFFSET (sqrt((START_Y*START_Y)/2.0))  // offset y coordinates for the 4 "corner" legs

// below are measured from the center of the body to the point at which the coxa pivots at the body
#define Y_CORNER_OFFSET 4.7 // cm from center in the forward/backward axis
#define X_CORNER_OFFSET 7.4 // cm from center in the left/right axis
#define Y_CENTER_OFFSET 6.8

extern int legs[6][3];

struct position
{
    float angle1; // servo at the inner most end of the leg
    float angle2; // middle servo
    float angle3; // servo at the outter most end of the leg
};

struct coordinate
{
	float x;
	float y;
	float z;
};

struct motor_status
{
    float position;
    float speed;
};

struct leg_status
{
    struct motor_status motors[3];
};

double sq(double x);
float get_gamma(float x, float y);
int get_angles(struct position* pos, struct coordinate* coord);
int get_angles_relative(int leg_num, struct position* pos, struct coordinate* coord);
int get_position(struct coordinate* coord, struct position* pos);
void get_position_relative(int leg_num, struct coordinate* coord, struct position* pos);
void get_leg_status(int leg_num, struct leg_status* leg_stat);
int get_rotate_location_relative(int leg_num, struct coordinate* coord, double theta, double r);
int move_leg(int leg_num, struct coordinate* coord);
int move_leg_relative(int leg_num, struct coordinate* coord);
int get_current_leg_position(int leg_num, struct coordinate* coord);
//int position_legs_start();
int reposition_legs();
void until_legs_stop();
double to_degrees(double radians);
double to_radians(double degrees);

#endif
