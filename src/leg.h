#ifndef _ANGLES_H_
#define _ANGLES_H_

#define COXA 5.2
#define FEMUR 8.3
#define TIBIA 17
#define ZOFFSET 8.5 // !! need to measure/set this

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

int get_angles(struct position* pos, struct coordinate* coord);
int move_leg(int leg_num, struct coordinate* coord);
int move_leg_relative(int leg_num, struct coordinate* coord, float i);

#endif
