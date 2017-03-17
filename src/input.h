#ifndef _INPUT_H_
#define _INPUT_H_
#include "leg.h"

#define DEADZONE 0.02   //the percentage of area around the center of the joystick that evaluates to zero input
#define RATE 17         //max linear speed in cm/sec at full stick inputs

struct controller
{
    int select;
    int start;
    int ps_button;
    int triangle;
    int circle;
    int x;
    int square;
    float d_up;
    float d_right;
    float d_down;
    float d_left;
    int l2;
    int r2;
    int l1;
    int r1;
    float left_joy_x;
    float left_joy_y;
    int left_joy_click;
    float right_joy_x;
    float right_joy_y;
    int right_joy_click;
    float d_x;
    float d_y;
};

double max(double x, double y);
int openController(struct controller* control);
void getPresses(struct controller* control);
void getDifference(struct coordinate* coord, double x, double y, double time);
int getWalkDiff(struct coordinate* new_coord, struct coordinate* old_coord, double x_axis, double y_axis, double radius, double time);
void getAbsolute(struct coordinate* coord, double x_axis, double y_axis, double z_axis);
void closeController();

#endif
