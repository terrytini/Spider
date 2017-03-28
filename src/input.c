#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <sys/select.h>
#include <stdlib.h>
#include "input.h"

static int controller_fd;
static unsigned char buf[32];

// returns the maximum of the two given doubles
double max(double x, double y)
{
    if (x>y)
        return x;
    else
        return y;
}

// opens controller port and initializes controller struct (if provided)
int openController(struct controller* control)
{

    controller_fd = open("/dev/hidraw1", O_RDONLY); //   for bluetooth -> "/dev/hidraw2"
                                                    //   I can't get usb to work, but should be -> "/dev/usb/hiddev0"

    if (control != NULL)
    {
        control->select = 0;
        control->left_joy_click = 0;
        control->right_joy_click =  0;
        control->start =  0;
        control->l2 = 0;
        control->r2 = 0;
        control->l1 = 0;
        control->r1 = 0;
        control->triangle = 0;
        control->circle = 0;
        control->x = 0;
        control->square = 0;
        control->ps_button =  0;
        control->left_joy_x = 0;
        control->left_joy_y = 0;
        control->right_joy_x = 0;
        control->right_joy_y = 0;
        control->d_up = 0;
        control->d_right = 0;
        control->d_down = 0;
        control->d_left = 0;
        control->d_x = 0;
        control->d_y = 0;
    }

    if(controller_fd < 1)
    {
        perror("Failed to open the controller device");
        return -1;
    }
    else
    {
        int flags = fcntl(controller_fd, F_GETFL, 0);
        fcntl(controller_fd, F_SETFL, flags | O_NONBLOCK);
        return controller_fd;
    }
}

// normalizes a joy stick input around the origin and accounts for deadzone
double normalize(double input)
{
    if (input <= 0.5 + DEADZONE && input >= 0.5 - DEADZONE)
    {
        input = 0;
    }
    else
    {
        if (input < 0.5 - DEADZONE)
        {
            input = (input / (0.5 - DEADZONE)) - 1;
        }
        else //control->left_joy_x > 0.5 + DEADZONE
        {
            input = (input-(0.5+DEADZONE)) / (0.5 - DEADZONE);
        }
    }
    return input;
}

// updates a controller struct with the most recent button presses
void getPresses(struct controller* control)
{


    /* !! This would be more efficient if it used the select() call but i could not get it to work...
     do
     {
     ret = select(1, &rfds, NULL, NULL, &tv);
     //printf("ret: %d\n", ret);
     read(controller_fd, &buf, 32);
     }while(ret);*/

    while (read(controller_fd, &buf, 32) != -1);

    control->select = buf[2] & 1;
    control->left_joy_click = buf[2]>>1 & 1;
    control->right_joy_click =  buf[2]>>2 & 1;
    control->start =  buf[2]>>3 & 1;
    control->l2 = buf[3] & 1;
    control->r2 = buf[3]>>1 & 1;
    control->l1 = buf[3]>>2 & 1;
    control->r1 = buf[3]>>3 & 1;
    control->triangle = buf[3]>>4 & 1;
    control->circle = buf[3]>>5 & 1;
    control->x = buf[3]>>6 & 1;
    control->square = buf[3]>>7 & 1;
    control->ps_button =  buf[4] & 1;
    control->left_joy_x = buf[6]/255.0;
    control->left_joy_y = (255-buf[7])/255.0;
    control->right_joy_x = buf[8]/255.0;
    control->right_joy_y = (255-buf[9])/255.0;
    control->d_up = buf[14]/255.0;
    control->d_right = buf[15]/255.0;
    control->d_down = buf[16]/255.0;
    control->d_left = buf[17]/255.0;

    // account for deadzone (the joysticks are never ACTUALLY centered, this is to make sure they are at zero value when within DEADZONE of centered)
    control->left_joy_x = normalize(control->left_joy_x);
    control->left_joy_y = normalize(control->left_joy_y);
    control->right_joy_x = normalize(control->right_joy_x);
    control->right_joy_y = normalize(control->right_joy_y);

    if(control->d_down > control->d_up)
        control->d_y = -control->d_down;
    else
        control->d_y = control->d_up;

    if(control->d_left > control->d_right)
        control->d_x = -control->d_left;
    else
        control->d_x = control->d_right;

    // these two lines below flip the controls as if the robots front is now its rear
    //control->right_joy_x = -control->right_joy_x;
    //control->right_joy_y = -control->right_joy_y;

    //printf("ljx:%f ljy:%f rjx:%f rjy:%f d_x:%f d_y:%f\n", control->left_joy_x,control->left_joy_y,control->right_joy_x,control->right_joy_y, control->d_x, control->d_y);
}

// gets the difference in x and y coordinates relative to joystick inputs given some elapsed time
void getDifference(struct coordinate* coord, double x_axis, double y_axis, double time)
{
    double hypotenuse = RATE * time * max(fabsf(x_axis), fabsf(y_axis));
    coord->z = 0;   // !! probably shouldn't do this?

    //check for infinity slope
    if (y_axis==0)
    {
        if (x_axis>=0)
        {
            coord->y = hypotenuse;
            coord->x = 0;
        }
        else
        {
            coord->y = -hypotenuse;
            coord->x = 0;
        }
    }
    //check for 0 slope
    else if(x_axis==0)
    {
        if (y_axis>=0)
        {
            coord->y = 0;
            coord->x = hypotenuse;
        }
        else
        {
            coord->y = 0;
            coord->x = -hypotenuse;
        }
    }
    else
    {
        //calculate x and y given slope and hypotenuse
        double slope = y_axis/-x_axis;
        coord->y = hypotenuse/sqrt(sq(slope)+1);
        coord->x = coord->y * slope;

        if (x_axis<0)
        {
            coord->y = -coord->y;
        }
        else
            coord->x = -coord->x;
    }
}

// given x and y vector coordinates, returns the angle from the origin
double get_theta(double x_input, double y_input)
{
    double theta = 0;
    if (x_input != 0)
    {
        theta = atan(fabsf(y_input/x_input));
        if (y_input<0 && x_input<0)
            theta+=to_radians(180);
        else if (y_input<0)
            theta = to_radians(360) - theta;
        else if (x_input<0)
            theta = to_radians(180) - theta;
    }
    else
    {
        if (y_input > 0)
            theta = to_radians(90);
        else
            theta = to_radians(270);
    }
    return theta;
}

// TODO - also calculate z below and in others
// gets the difference in x and y coordinates for walking/stepping, relative to joystick inputs, given some elapsed time
// calculations are made with leg#4's coodinate plane in mind
int getWalkDiff(struct coordinate* new_coord, struct coordinate* old_coord, double x_input, double y_input, double radius, double time)
{
    // calculate point along circle from inputs and boundary (radius)
    // the inputs, and leg coord. plane are rotated 90 degrees from one another (hence x=y and y=-x)
    //double y_input = -x_axis;
    //double x_input = y_axis;
    double old_x = old_coord->y - START_Y;
    double old_y = -old_coord->x;

    //printf("inputs: x: %f\ty: %f\n", x_input, y_input);
    //printf("old: x: %f\t y: %f\n", old_x, old_y);
    double theta = get_theta(x_input, y_input);

    double circum_x = cos(theta) * radius;
    double circum_y = sin(theta) * radius;

    //printf("5 circum_x: %f\tcircum_y: %f\ttheta: %f\n", circum_x, circum_y, theta);

    // calculate point along line between old_coord and point along circumference of circle
    // calculate vector from the two points (so line begins at origin)
    double vect_x = circum_x - old_x;
    double vect_y = circum_y - old_y;
    //printf("6 vect_x: %f\tvect_y: %f\n",vect_x, vect_y);

    // calculate point along vector RATE*time*magnitude from origin
    theta = get_theta(vect_x, vect_y);//atan(vect_y/vect_x);
    double magnitude = max(fabsf(x_input), fabsf(y_input));
    double hyp = RATE*time*magnitude;
    vect_x = cos(theta) * hyp;
    vect_y = sin(theta) * hyp;

    //printf("7 vect_x: %f\tvect_y: %f\thyp: %f\n",vect_x, vect_y, hyp);

    // add back old_coords to caclulate the actual location of the point we want the leg to be at
    //printf("old_coords: x= %f\ty=%f\n", old_coord->x,old_coord->y);
    new_coord->x = vect_x + old_x;
    new_coord->y = vect_y + old_y;
    //printf("8 new_coord->x: %f\tnew_coord->y: %f\n",new_coord->x, new_coord->y);

    // if the point returned lies outside of the radius from the origin, return the coordinates along the circumference, circum_x and circum_y times the radius
    if ((sqrt(sq(new_coord->x)+sq(new_coord->y)) >= radius + 0.000001) && (sq(new_coord->x)+sq(new_coord->y) > 0))
    {
        new_coord->y = circum_x;
        new_coord->x = -circum_y;

        // return 1 if we have reached the boundary
        return 1;
    }
    else
    {
        double temp_x = new_coord->x;
        new_coord->x = -new_coord->y;
        new_coord->y = temp_x;
        // return 0 if boundary has not yet been reached
        //printf("9 new_coord->x: %f\tnew_coord->y: %f\tnew_coord->z: %f\n",new_coord->x, new_coord->y, new_coord->z);
        return 0;
    }
}

// calculates the difference in x and y coordinates from the origin (at rest position)
// for a given leg, relative to joystick inputs
void getAbsolute(struct coordinate* coord, double x_axis, double y_axis, double z_axis)
{
    y_axis = -y_axis; //to "calibrate" direction [to leg #4?]
    double hypotenuse = max(fabs(x_axis), fabs(y_axis));
    double boundary = 9; //!! this is for testing; TODO - define a constant in header file
    double z_boundary = 4;
    //check for infinity slope
    if (y_axis==0)
    {
        if (x_axis>=0)
        {
            coord->y = hypotenuse;
            coord->x = 0;
        }
        else
        {
            coord->y = -hypotenuse;
            coord->x = 0;
        }
    }
    //check for 0 slope
    else if(x_axis==0)
    {
        if (y_axis>=0)
        {
            coord->y = 0;
            coord->x = hypotenuse;
        }
        else
        {
            coord->y = 0;
            coord->x = -hypotenuse;
        }
    }
    else
    {
        //calculate x and y given slope and hypotenuse
        double slope = y_axis/-x_axis;
        coord->y = hypotenuse/sqrt(sq(slope)+1);
        coord->x = coord->y * slope;

        if (x_axis<0)
        {
            coord->y = -coord->y;
        }
        else
            coord->x = -coord->x;
    }

    coord->x*=boundary;
    coord->y*=boundary;
    coord->z = z_axis;//z_boundary*z_axis;   // !! z boundary should be set elsewhere (option# 11 takes care of this - 10 still needs work)
}

void closeController()
{
    close(controller_fd);
}

// used for testing :
/*int main(void)
 {
 struct controller cont;
 cont.ps_button = 0;

 if (openController(&cont) != 0)
 return 1;

 while(cont.ps_button != 1)
 {

 getPresses(&cont);
 printf("sel:%d sta:%d ps:%d lc:%d rc:%d \n", cont.select,cont.start,cont.ps_button,cont.left_joy_click,cont.right_joy_click);
 printf("t:%d c:%d x:%d s:%d l2:%d r2:%d l1:%d r1:%d\n", cont.triangle, cont.circle,cont.x, cont.square,cont.l2,cont.r2,cont.l1,cont.r1);
 printf("ljx:%f ljy:%f rjx:%f rjy:%f\n", cont.left_joy_x,cont.left_joy_y,cont.right_joy_x,cont.right_joy_y);
 printf("d_x: %f d_y: %f up:%f right:%f down:%f left:%f\n", cont.d_x, cont.d_y, cont.d_up,cont.d_right,cont.d_down,cont.d_left);
 usleep(100000);
 }

 close(controller_fd);
 return 0;
 }*/
