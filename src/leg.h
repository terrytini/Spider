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

int move_leg(int leg_num, float x, float y, float z);
int get_angles(struct position* pos, float x, float y, float z, int leg_num);

#endif
