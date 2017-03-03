#ifndef _LEGS_H_
#define _LEGS_H_

#define COXA 5.2
#define FEMUR 8.3
#define TIBIA 17
#define ZOFFSET 11.5 //11.5 //8.5
#define START_X 0.0    // starting (relaxed) x coordinate for any leg
#define START_Y 17.0   // starting (relaxed) y coordinate for any leg
#define START_Z 0.0    // starting (relaxed) z coordinate for any leg

//extern float ZOFFSET = 8.5;   //!! TODO change final (needs refactored because it was a "#define" statement above)
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

#endif
