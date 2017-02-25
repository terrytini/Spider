#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <sys/select.h>
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
    
    controller_fd = open("/dev/hidraw2", O_RDONLY);
    
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
        perror("Failed to open the device");
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
    
    
    //printf("ljx:%f ljy:%f rjx:%f rjy:%f d_x:%f d_y:%f\n", control->left_joy_x,control->left_joy_y,control->right_joy_x,control->right_joy_y, control->d_x, control->d_y);
}

// gets the difference in x and y coordinates relative to joystick inputs given some elapsed time
void getDifference(struct coordinate* coord, double x_axis, double y_axis, double time)
{
    double hypotenuse = RATE * time * max(fabsf(x_axis), fabsf(y_axis));
    coord->z = 0;
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
    coord->z = z_boundary*z_axis;   // !! z boundary shoud be set elsewhere (option# 11 takes care of this - 10 still needs work)
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
