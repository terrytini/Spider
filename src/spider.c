#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "leg.h"
#include "ax12a.h"
#include "input.h"

static int mode = 1;                // this is like the current/starting "gate"
static int option = 0;			    // this is the current/starting option within each gate
static struct controller control;	// holds the button presses (must call get_presses() on it to update)

// returns 1 if menu button is pressed on the controller
int menu_pressed()
{
    if (control.x || control.circle || control.triangle)
        return 1;
    else
        return 0;
}

// returns 1 if option button is pressed on the controller
int option_pressed()
{
    if (mode == 0 && (control.right_joy_click || control.left_joy_click))
        return 1;
    else if (mode == 1 && option == 0 && !control.right_joy_x && !control.right_joy_y && !control.d_y && !control.d_x && control.left_joy_x)
    // if we are in walking mode and none of the walking buttons are pressed, but the rotate button is pressed -> switch to rotate mode
        return 1;
    else if (mode == 1 && option == 1 && !control.left_joy_y && (control.right_joy_x || control.right_joy_y || control.d_x || control.d_y))
    // if we are in rotate mode and none of the rotate buttons are pressed, but a walking button is pressed -> switch to walk mode
        return 1;
    else if (mode == 2 && (control.left_joy_click || control.right_joy_click))
        return 1;
    else
        return 0;
}

// changes menu options and returns 1 if a menu/option button is pressed, 0 otherwise
int get_selection()
{
    // see if a menu button is pressed
    if (control.circle)
        mode = 0;   // circle button is "control body" mode
    else if (control.x)
    {
        option = 0;
        mode = 1;   // x button is "walking" mode
    }
    else if (control.triangle)
    {
        option = 5;
        mode = 2;   // triangle button is "control leg" mode
    }
    else if (mode == 1 && option == 0 && !control.right_joy_x && !control.right_joy_y && !control.d_y && !control.d_x && control.left_joy_x)
    // if we are in walking mode and none of the walking buttons are pressed, but the rotate button is pressed -> switch to rotate mode
        option = 1;
    else if (mode == 1 && option == 1 && !control.left_joy_y && (control.right_joy_x || control.right_joy_y || control.d_x || control.d_y))
    // if we are in rotate mode and none of the rotate buttons are pressed, but a walking button is pressed -> switch to walk mode
        option = 0;
    else if (mode == 2 && control.left_joy_click)
        //click down on joysticks in control leg mode to switch the current leg
        option  = (option + 1) % 6;
    else if (mode == 2 && control.right_joy_click)
    {
        option = (option - 1) % 6;
        if (option < 0)
          option = option + 6;
    }
    else
        return 0;

    // wait until button is released
    while (menu_pressed() || option_pressed())
        getPresses(&control);

    printf("mode: %d\toption: %d\n", mode, option);

    return 1;
}

// allows for absolute control of the coordinates of the center of the body using the controller
void control_body()
{
    double P = 0.5;
    double boundary = 10;           // 1/2 of the full stride (radius of leg movement in x-y plane)
    double diff1, diff2, diff3;     // abs vals of differences between the current and desired angles for servos 1, 2, and 3 respectively
    double speed1, speed2, speed3;  // speed values to turn each servo motor at
    double z_input;                 // the controller input value for the z_axis
    double z_up_bound = 9, z_down_bound = 7.5;// the maximum movement that the body can make up or down from START_Z in the z-axis
    struct leg_status leg_stat;     // holds the status (speeds and positions) of each motor on a leg
    struct coordinate desired_coord, actual_coord, diff_coord;
    struct position desired_pos, actual_pos;
    //struct controller control;      // holds the button presses (must call get_presses() on it to update)
    double elapsed;                 // amount of time that elapsed since last update
    struct timeval tval_start, tval_now;//!! don't need this?
    desired_coord.x = START_X;      // The starting coordinates of the legs
    desired_coord.y = START_Y;
    desired_coord.z = START_Z;

    // position legs to their starting (relaxed) position
    reposition_legs();

    // control loop

    while (!menu_pressed())
    {
        // find out where the servos are
        for (int leg_num = 0; leg_num<6; leg_num++)
        {
            get_leg_status(leg_num, &leg_stat);

            actual_pos.angle1 = leg_stat.motors[0].position;    // !! TODO - we may have to change this to do the adjustment between reading from each motor in order to minimize delay
            actual_pos.angle2 = leg_stat.motors[1].position;
            actual_pos.angle3 = leg_stat.motors[2].position;

            //get the coordinates of the tip of the leg
            get_position_relative(leg_num, &actual_coord, &actual_pos);
            //printf("actual coord: x:%f\ty:%f\tz:%f\n", actual_coord.x, actual_coord.y, actual_coord.z);

            // calculate where the servos should be
            // get stick inputs
            getPresses(&control);
            gettimeofday(&tval_now, NULL);
            elapsed = (tval_now.tv_sec - tval_start.tv_sec) + (tval_now.tv_usec - tval_start.tv_usec)/1000000.0;
            tval_start.tv_sec = tval_now.tv_sec;
            tval_start.tv_usec = tval_now.tv_usec;

            //instead of getting difference here, just get absolute position (relative to joystick inputs) - no need for timer
            // let d-pad OR left joystick be able to control z axis (whichever receives most extreme input)
            if(fabsf(control.d_y)>fabsf(control.left_joy_y))
                z_input = control.d_y;
            else
                z_input = control.left_joy_y;

            getAbsolute(&desired_coord, -control.right_joy_x, -control.right_joy_y, -z_input);

            // adjust x and y inputs according to z_input (only let body move in sphere rather than cylinder in 3-D space)
            desired_coord.x = desired_coord.x * (fabsf(0.67 * (1-fabsf(z_input))) + 0.33);   // !! adjust what we are subtracting the abs val from here to allow a little movement at extreme z-coords
            desired_coord.y = desired_coord.y * (fabsf(0.67 * (1-fabsf(z_input))) + 0.33);

            if (-z_input < 0)
                desired_coord.z = -z_input * z_up_bound;
            else
                desired_coord.z = -z_input * z_down_bound;

            if (leg_num/3 < 1)
            {
                // if the leg is on the left side of the body, flip the inputs (x and y axes are rotated 180 degrees)
                desired_coord.x = -desired_coord.x;
                desired_coord.y = -desired_coord.y;
            }

            desired_coord.y += START_Y;

            //desired_coord.x = desired_coord.x + diff_coord.x;
            //desired_coord.y = desired_coord.y + diff_coord.y;
            //desired_coord.z = desired_coord.z + diff_coord.z;

            get_angles_relative(leg_num, &desired_pos, &desired_coord);

            //printf("actual ang: %f\t%f\t%f\n", actual_pos.angle1, actual_pos.angle2, actual_pos.angle3);
            //printf("desired: %f\t%f\t%f\n", desired_pos.angle1, desired_pos.angle2, desired_pos.angle3);

            // calculate how far we are from where we need to be
            diff1 = fabsf(actual_pos.angle1 - desired_pos.angle1);
            diff2 = fabsf(actual_pos.angle2 - desired_pos.angle2);
            diff3 = fabsf(actual_pos.angle3 - desired_pos.angle3);
            //printf("diffs: %f\t%f\t%f\n", diff1, diff2, diff3);

            // TODO - try uncommenting below and see how it works? - BE CAREFUL...
            // travel CW or CCW rather than to a specific target angle
            /*if (coord.x != boundary)
             {
             if (actual_pos.angle1 - desired_pos.angle1 < 0)
             desired_pos.angle1 = MAX_ANGLE;
             else if (actual_pos.angle1 - desired_pos.angle1 > 0)
             desired_pos.angle1 = MIN_ANGLE;

             if (actual_pos.angle2 - desired_pos.angle2 < 0)
             desired_pos.angle2 = MAX_ANGLE;
             else if (actual_pos.angle2 - desired_pos.angle2 > 0)
             desired_pos.angle2 = MIN_ANGLE;

             if (actual_pos.angle3 - desired_pos.angle3 < 0)
             desired_pos.angle3 = MAX_ANGLE;
             else if (actual_pos.angle3 - desired_pos.angle3 > 0)
             desired_pos.angle3 = MIN_ANGLE;
             }*/

            // diffs are abs vals, so must be >0, we need to ensure they are >1 because 1 is the minimum speed of
            // the servos (less than 1 is apparently max speed)
            speed1 = P*diff1 + 1;
            speed2 = P*diff2 + 1;
            speed3 = P*diff3 + 1;

            //printf("speeds = %f\t%f\t%f\n", speed1, speed2, speed3);

            // adjust accordingly
            turnMotor(legs[leg_num][0], desired_pos.angle1, speed1);
            turnMotor(legs[leg_num][1], desired_pos.angle2, speed2);
            turnMotor(legs[leg_num][2], desired_pos.angle3, speed3);

            if (menu_pressed())
                break;
        }
    }
}

void walk()
{
    double P = 0.47;
    double boundary = 7;            // 1/2 of the full stride (radius of leg movement in x-y plane)
    double diff1, diff2, diff3;     // abs vals of differences between the current and desired angles for servos 1, 2, and 3 respectively
    double speed1, speed2, speed3;  // speed values to turn each servo motor at
    double x_input, y_input, z_input;                 // the controller input values for each axis
    double z_up_bound = 9, z_down_bound = 7.5;// the maximum movement that the body can make up or down from zero in the z-axis
    struct leg_status leg_stat;     // holds the statuses (speeds and positions) for each motor on a leg
    struct coordinate desired_coord, actual_coord, diff_coord, local_coord;
    struct position desired_pos, actual_pos;
    int change = 0;                   // flag variable for when to change direction
    double elapsed;                 // amount of time that elapsed since last update
    struct timeval tval_start, tval_now;
    int direction = 1;
    int flip = 0;                   // flag variable indicating when we should step (i.e. change direction)
    desired_coord.x = START_X;      // the starting coordinates of the legs
    desired_coord.y = START_Y;
    desired_coord.z = START_Z;

    // position legs to their starting (relaxed) position
    reposition_legs();

    // !! TODO - maybe try picking up opposing legs here before entering loop?

    gettimeofday(&tval_start, NULL);

    // control loop
    while(!menu_pressed() && !option_pressed())
    {
        for (int leg_num = 0; leg_num<6; leg_num++)
        {
            // find out where the servos are
            get_leg_status(leg_num, &leg_stat);

            actual_pos.angle1 = leg_stat.motors[0].position;    // !! TODO - we may have to change this to do the adjustment between reading from each motor in order to minimize delay
            actual_pos.angle2 = leg_stat.motors[1].position;
            actual_pos.angle3 = leg_stat.motors[2].position;

            //get the coordinates of the tip of the leg
            get_position_relative(leg_num, &actual_coord, &actual_pos);
            //printf("actual coord: x:%f\ty:%f\tz:%f\n", actual_coord.x, actual_coord.y, actual_coord.z);

            // get stick inputs
            getPresses(&control);

            // let d-pad OR right joystick control motion (whichever receives most extreme input)
            if(max(fabsf(control.d_x), fabsf(control.d_y))>max(fabsf(control.right_joy_x), fabsf(control.right_joy_y)))
            {
                x_input = control.d_x;
                y_input = control.d_y;
            }
            else
            {
                x_input = control.right_joy_x;
                y_input = control.right_joy_y;
            }

            if (direction < 0)
            {
                //reverse inputs if direction is reversed
                x_input = -x_input;
                y_input = -y_input;
            }

            // get elapsed time since last loop iteration
            gettimeofday(&tval_now, NULL);
            elapsed = (tval_now.tv_sec - tval_start.tv_sec) + (tval_now.tv_usec - tval_start.tv_usec)/1000000.0;
            tval_start.tv_sec = tval_now.tv_sec;
            tval_start.tv_usec = tval_now.tv_usec;

            // get the coordinates of where the leg should be (legs on left side of body will need reversed)
            //getDifference(&diff_coord, control.right_joy_x, -control.right_joy_y, elapsed);

            if (getWalkDiff(&diff_coord, &desired_coord, -x_input, -y_input, boundary, elapsed))
                    flip = 1;

            //printf("flip: %d\n", flip);
            //printf("1 diff_coord= x: %f y: %f z: %f\n", diff_coord.x, diff_coord.y, diff_coord.z);

            // update desired_coord to the new desired point
            desired_coord.x = diff_coord.x;
            desired_coord.y = diff_coord.y;
            // !! desired_coord.z = diff_coord.z;

            //printf("2 desired_coord= x: %f y: %f z: %f\n", desired_coord.x, desired_coord.y, desired_coord.z);

            // flip coordinates for legs on opposite side
            if (leg_num/3 < 1)
            {
                local_coord.x = -desired_coord.x;
                local_coord.y = -desired_coord.y;
            }
            else
            {
                local_coord.x = desired_coord.x;
                local_coord.y = desired_coord.y;
            }

            // flip coordinates for legs that are touching the ground - this is unnessesary to flip, because they flip when changing direction...
            int on_ground;

            if (direction > 0)
                on_ground = (leg_num % 2) == 1;
            else
            {
                local_coord.x = -local_coord.x;
                local_coord.y = -local_coord.y;
                on_ground = (leg_num % 2) == 0;
            }

            if (!on_ground && !flip)
            {
                // lift legs that are not on the ground
                local_coord.z = STEP_Z;
            }
            else
                local_coord.z = START_Z;

            if (!on_ground)
            {
                local_coord.x = -local_coord.x;
                local_coord.y = -local_coord.y;
            }

            // adjust y_coord to START_Y offset
            desired_coord.y += START_Y;
            local_coord.y += START_Y;


            get_angles_relative(leg_num, &desired_pos, &local_coord);

            //printf("actual ang: %f\t%f\t%f\n", actual_pos.angle1, actual_pos.angle2, actual_pos.angle3);
            //printf("desired: %f\t%f\t%f\n", desired_pos.angle1, desired_pos.angle2, desired_pos.angle3);

            // calculate how far we are from where we need to be
            diff1 = fabsf(actual_pos.angle1 - desired_pos.angle1);
            diff2 = fabsf(actual_pos.angle2 - desired_pos.angle2);
            diff3 = fabsf(actual_pos.angle3 - desired_pos.angle3);
            //printf("diffs: %f\t%f\t%f\n", diff1, diff2, diff3);

            // travel CW or CCW rather than to a specific target angle
            /*if (coord.x != boundary)
             {
             if (actual_pos.angle1 - desired_pos.angle1 < 0)
             desired_pos.angle1 = MAX_ANGLE;
             else if (actual_pos.angle1 - desired_pos.angle1 > 0)
             desired_pos.angle1 = MIN_ANGLE;

             if (actual_pos.angle2 - desired_pos.angle2 < 0)
             desired_pos.angle2 = MAX_ANGLE;
             else if (actual_pos.angle2 - desired_pos.angle2 > 0)
             desired_pos.angle2 = MIN_ANGLE;

             if (actual_pos.angle3 - desired_pos.angle3 < 0)
             desired_pos.angle3 = MAX_ANGLE;
             else if (actual_pos.angle3 - desired_pos.angle3 > 0)
             desired_pos.angle3 = MIN_ANGLE;
             }*/

            // make sure that the speed values cannot be less than 1 (full speed if already near desired position = BAD)
            speed1 = P*diff1 + 1;
            speed2 = P*diff2 + 1;
            speed3 = P*diff3 + 1;

            //printf("speeds = %f\t%f\t%f\n", speed1, speed2, speed3);

            // adjust accordingly
            turnMotor(legs[leg_num][0], desired_pos.angle1, speed1);
            turnMotor(legs[leg_num][1], desired_pos.angle2, speed2);
            turnMotor(legs[leg_num][2], desired_pos.angle3, speed3);

            if (menu_pressed() || option_pressed())
                break;
        }
        // change direction
        if (flip)
        {
            // even when we want to change direction, we iterate the above for loop once more, to ensure all 6 legs are on the ground, and have reached the boundary
            change = 1;
        }
        if (change)
        {
            // !! TODO - maybe try telling/waiting for all legs to reach boundary here? - if last leg in for loop above was to reach boundary, others would fall short before changing direction
            //until_legs_stop();
            flip = 0;
            change = 0;
            direction = -direction;
            //printf("stepping\n");
        }
    }
}

void rotate()
{
    double P = 0.47;
    double boundary = 10;           // 1/2 of the full stride (radius of leg movement in x-y plane)
    double diff1, diff2, diff3;     // abs vals of differences between the current and desired angles for servos 1, 2, and 3 respectively
    double speed1, speed2, speed3;  // speed values to turn each servo motor at
    double z_input;                 // the controller input value for the z_axis
    double z_up_bound = 9, z_down_bound = 8;// the maximum movement that the body can make up or down from zero in the z-axis
    struct leg_status leg_stat;     // holds the statuses (speeds and positions) for each motor on a leg
    struct coordinate desired_coord, actual_coord, diff_coord;
    struct position desired_pos, actual_pos;
    double theta = 0;
    double max_dps = 70;        // maximum degrees per second that the body can rotate at
    double dps = 0;             // current rotation speed of body (degrees per second)
    double max_theta = 21;      // the maximum amount of rotation in either direction from center
    double elapsed;             // amount of time that elapsed since last update
    struct timeval tval_start, tval_now;
    int direction = 1;          // this dictates the direction of leading legs
    desired_coord.x = START_X;  // the starting coordinates of the legs
    desired_coord.y = START_Y;
    desired_coord.z = START_Z;

    // position legs to their starting (relaxed) position
    reposition_legs();

    // control loop
    while(!menu_pressed() && !option_pressed())
    {

        gettimeofday(&tval_start, NULL);

        while (fabsf(theta) < max_theta && !menu_pressed() && !option_pressed())//(sqrt(sq(desired_coord.x)+sq(desired_coord.y-15)) < boundary && !control.ps_button)  //!! either != boundary (and TODO - set boundary to boundary if outside of it) OR < boundary
        {
            // find out where the servos are
            for (int leg_num = 0; leg_num<6; leg_num++)
            {

                get_leg_status(leg_num, &leg_stat);

                actual_pos.angle1 = leg_stat.motors[0].position;    // !! TODO - we may have to change this to do the adjustment between reading from each motor in order to minimize delay
                actual_pos.angle2 = leg_stat.motors[1].position;
                actual_pos.angle3 = leg_stat.motors[2].position;

                //get the coordinates of the tip of the leg
                get_position_relative(leg_num, &actual_coord, &actual_pos);
                //printf("actual coord: x:%f\ty:%f\tz:%f\n", actual_coord.x, actual_coord.y, actual_coord.z);

                // calculate where the servos should be
                // get stick inputs
                getPresses(&control);

                // let d-pad OR right joystick control rotation (whichever receives most extreme input)
                /*if(fabsf(control.d_x)>fabsf(control.right_joy_x))
                    z_input = control.d_x;
                else
                    z_input = control.right_joy_x;
                */

                dps = max_dps * control.left_joy_x;//z_input;

                //double intensity = max(fabsf(control.right_joy_x), fabsf(control.right_joy_y));
                gettimeofday(&tval_now, NULL);
                elapsed = (tval_now.tv_sec - tval_start.tv_sec) + (tval_now.tv_usec - tval_start.tv_usec)/1000000.0;
                tval_start.tv_sec = tval_now.tv_sec;    // !! maybe try doing these two lines after telling the motors to move??
                tval_start.tv_usec = tval_now.tv_usec;

                // calculate inputs to get_rotate_location_relative(leg_num, coord, theta, r)
                if (direction > 0)
                    theta = theta + dps * elapsed;   // !! this [may] need changed when moving more than 1 leg at a time?
                else
                    theta = theta - dps * elapsed;

                if (theta > max_theta)
                    theta = max_theta;
                else if (theta < -max_theta)
                    theta = -max_theta;

                double r = START_Y + Y_CENTER_OFFSET;


                // getAbsolute(&desired_coord, -control.right_joy_x, -control.right_joy_y, -z_input);
                // given leg number, angle from center (calculate from OFFSETS (both internal and external)), and radius of circle to trace with tips
                // of legs, calculates the x,y, and z coordinates that the tip of leg should be at
                //if (leg_num/3 < 1)
                int on_ground = ((direction > 0) && (leg_num % 2 == 0)) || ((direction < 0) && (leg_num % 2 == 1));   // 1 if the current leg is on the ground, 0 if it is lifted
                double local_theta = theta;
                if (on_ground)
                {
                    desired_coord.z = 0;
                    local_theta = -theta;//get_rotate_location_relative(leg_num, &desired_coord, theta, r);
                }
                else
                {
                    desired_coord.z = STEP_Z;
                    local_theta = -theta;//get_rotate_location_relative(leg_num, &desired_coord, -theta, r);
                }

                int dominant = (leg_num % 2) == 0;  // even numbered legs are "dominant" in tripod walk
                if (dominant)
                {
                    local_theta = -local_theta;
                }

                get_rotate_location_relative(leg_num, &desired_coord, local_theta, r);
                desired_coord.y += START_Y;

                //desired_coord.x = desired_coord.x + diff_coord.x;
                //desired_coord.y = desired_coord.y + diff_coord.y;
                //desired_coord.z = desired_coord.z + diff_coord.z;

                get_angles_relative(leg_num, &desired_pos, &desired_coord);

                //printf("actual ang: %f\t%f\t%f\n", actual_pos.angle1, actual_pos.angle2, actual_pos.angle3);
                //printf("desired: %f\t%f\t%f\n", desired_coord.x, desired_coord.y, desired_coord.z);

                // calculate how far we are from where we need to be
                diff1 = fabsf(actual_pos.angle1 - desired_pos.angle1);
                diff2 = fabsf(actual_pos.angle2 - desired_pos.angle2);
                diff3 = fabsf(actual_pos.angle3 - desired_pos.angle3);
                //printf("diffs: %f\t%f\t%f\n", diff1, diff2, diff3);

                // TODO - try uncommenting below and see how it works? - BE CAREFUL...
                // travel CW or CCW rather than to a specific target angle
                /*if (coord.x != boundary)
                 {
                 if (actual_pos.angle1 - desired_pos.angle1 < 0)
                 desired_pos.angle1 = MAX_ANGLE;
                 else if (actual_pos.angle1 - desired_pos.angle1 > 0)
                 desired_pos.angle1 = MIN_ANGLE;

                 if (actual_pos.angle2 - desired_pos.angle2 < 0)
                 desired_pos.angle2 = MAX_ANGLE;
                 else if (actual_pos.angle2 - desired_pos.angle2 > 0)
                 desired_pos.angle2 = MIN_ANGLE;

                 if (actual_pos.angle3 - desired_pos.angle3 < 0)
                 desired_pos.angle3 = MAX_ANGLE;
                 else if (actual_pos.angle3 - desired_pos.angle3 > 0)
                 desired_pos.angle3 = MIN_ANGLE;
                 }*/

                // diffs are abs vals, so must be >0, we need to ensure they are >1 because 1 is the minimum speed of
                // the servos (less than 1 is apparently max speed)
                speed1 = P*diff1 + 1;
                speed2 = P*diff2 + 1;
                speed3 = P*diff3 + 1;

                //printf("speeds = %f\t%f\t%f\n", speed1, speed2, speed3);

                // adjust accordingly
                turnMotor(legs[leg_num][0], desired_pos.angle1, speed1);
                turnMotor(legs[leg_num][1], desired_pos.angle2, speed2);
                turnMotor(legs[leg_num][2], desired_pos.angle3, speed3);

                if (menu_pressed() || option_pressed())
                    break;
            }
        }

        direction = -direction;

        if (theta < 0)
            theta = -(max_theta - 0.00001);
        else
            theta = max_theta - 0.00001;

        //printf("theta = %f\tfabsf(theta = %f\tdirection = %d\n)", theta, fabsf(theta), direction);
    }
}

// absolute control of leg with controller as input
void control_leg(int leg_num)
{
    double P = 0.95;
    double boundary = 8;           // 1/2 of the full stride (radius of leg movement in x-y plane)
    double diff1, diff2, diff3;     // abs vals of differences between the current and desired angles for servos 1, 2, and 3 respectively
    double speed1, speed2, speed3;   // speed values to turn each motor at
    double z_input;                 // the controller input value for the z_axis
    struct leg_status leg_stat;     // holds the statuses (speeds and positions) for each motor on a leg
    struct coordinate desired_coord, actual_coord, diff_coord;
    struct position desired_pos, actual_pos;
    double lift_height = 1.0;       // how high (in cm) a leg that is being controlled is compared to the other legs
    double z_up_bound = 1.1;          // how high the leg can be lifted from lift_height

    desired_coord.x = START_X;      // The starting coordinates of the legs
    desired_coord.y = START_Y;
    desired_coord.z = lift_height;

    reposition_legs();

    // position leg a small amount above its starting (relaxed) position
    move_leg_relative(leg_num, &desired_coord);
    for (int i=0; i<3; i++)
        waitUntilStop(legs[leg_num][i]);

    while (!menu_pressed() && !option_pressed())
    {
        // find out where the servos are
        get_leg_status(leg_num, &leg_stat);

        actual_pos.angle1 = leg_stat.motors[0].position;    // !! we may have to change this to do the adjustment between reading from each motor in order to minimize delay
        actual_pos.angle2 = leg_stat.motors[1].position;
        actual_pos.angle3 = leg_stat.motors[2].position;

        //get the coordinates of the tip of the leg
        get_position_relative(leg_num, &actual_coord, &actual_pos);
        //printf("actual coord: x:%f\ty:%f\tz:%f\n", actual_coord.x, actual_coord.y, actual_coord.z);

        // calculate where the servos should be
        // get stick inputs
        getPresses(&control);

        if(fabsf(control.d_y)>fabsf(control.left_joy_y))
            z_input = control.d_y;
        else
            z_input = control.left_joy_y;

        getAbsolute(&desired_coord, control.right_joy_x, control.right_joy_y, z_input);  // !! TODO - make sure this correctly accounts for the [dynamic] resting position of y (START_Y)

        if (leg_num/3 < 1)
        {
            // if the leg is on the left side of the body, flip the coordinates (x and y axes are rotated 180 degrees)
            desired_coord.x = -desired_coord.x;
            desired_coord.y = -desired_coord.y;
        }

        if (z_input < 0)
        {
            desired_coord.z = z_input * lift_height + lift_height;
        }
        else if (z_input == 0)
            desired_coord.z = lift_height;
        else
            desired_coord.z = z_input * z_up_bound + lift_height;

        desired_coord.y += START_Y;
        desired_coord.z += lift_height;
        //printf("desired x: %f\ty: %f\n", desired_coord.x, desired_coord.y);

        get_angles_relative(leg_num, &desired_pos, &desired_coord);

        //printf("actual ang: %f\t%f\t%f\n", actual_pos.angle1, actual_pos.angle2, actual_pos.angle3);
        //printf("desired: %f\t%f\t%f\n", desired_pos.angle1, desired_pos.angle2, desired_pos.angle3);

        // calculate how far we are from where we need to be
        diff1 = fabsf(actual_pos.angle1 - desired_pos.angle1);
        diff2 = fabsf(actual_pos.angle2 - desired_pos.angle2);
        diff3 = fabsf(actual_pos.angle3 - desired_pos.angle3);
        //printf("diffs: %f\t%f\t%f\n", diff1, diff2, diff3);

        // TODO - try uncommenting below and see how it works? - BE CAREFUL...
        // travel CW or CCW rather than to a specific target angle
        /*if (coord.x != boundary)
         {
         if (actual_pos.angle1 - desired_pos.angle1 < 0)
         desired_pos.angle1 = MAX_ANGLE;
         else if (actual_pos.angle1 - desired_pos.angle1 > 0)
         desired_pos.angle1 = MIN_ANGLE;

         if (actual_pos.angle2 - desired_pos.angle2 < 0)
         desired_pos.angle2 = MAX_ANGLE;
         else if (actual_pos.angle2 - desired_pos.angle2 > 0)
         desired_pos.angle2 = MIN_ANGLE;

         if (actual_pos.angle3 - desired_pos.angle3 < 0)
         desired_pos.angle3 = MAX_ANGLE;
         else if (actual_pos.angle3 - desired_pos.angle3 > 0)
         desired_pos.angle3 = MIN_ANGLE;
         }*/

        // make sure that the speed values cannot be zero (full speed if already near desired position = BAD)
        // diffs are abs vals, so must be >0, we need to ensure they are >1 because 1 is the minimum speed of
        // the servos (less than 1 is apparently max speed)
        speed1 = P*diff1 + 1;
        speed2 = P*diff2 + 1;
        speed3 = P*diff3 + 1;

        //printf("speeds = %f\t%f\t%f\n", speed1, speed2, speed3);

        // adjust accordingly
        turnMotor(legs[leg_num][0], desired_pos.angle1, speed1);
        turnMotor(legs[leg_num][1], desired_pos.angle2, speed2);
        turnMotor(legs[leg_num][2], desired_pos.angle3, speed3);
    }
}

void display_desired()
{
    double P = 0.47;
    double boundary = 7;           // 1/2 of the full stride (radius of leg movement in x-y plane)
    double diff1, diff2, diff3;     // abs vals of differences between the current and desired angles for servos 1, 2, and 3 respectively
    double speed1, speed2, speed3;  // speed values to turn each servo motor at
    double z_input;                 // the controller input value for the z_axis
    double z_up_bound = 9, z_down_bound = 7.5;// the maximum movement that the body can make up or down from zero in the z-axis
    struct leg_status leg_stat;     // holds the statuses (speeds and positions) for each motor on a leg
    struct coordinate desired_coord, actual_coord, diff_coord, local_coord;
    struct position desired_pos, actual_pos;
    int change = 0;                   // flag variable for when to chagne direction
    //struct controller control;      // holds the button presses (must call get_presses() on it to update)

    double elapsed;                 // amount of time that elapsed since last update
    struct timeval tval_start, tval_now;
    int direction = 1;
    int flip = 0;                   // flag variable indicating when we should step (i.e. change direction)
    int firstloop = 1;
    desired_coord.x = START_X;      // the starting coordinates of the legs
    desired_coord.y = START_Y;
    desired_coord.z = START_Z;

    gettimeofday(&tval_start, NULL);

    // control loop
    while(!menu_pressed() && !option_pressed())
    {
            // get stick inputs
            getPresses(&control);

            // get elapsed time since last loop iteration
            gettimeofday(&tval_now, NULL);
            elapsed = (tval_now.tv_sec - tval_start.tv_sec) + (tval_now.tv_usec - tval_start.tv_usec)/1000000.0;
            tval_start.tv_sec = tval_now.tv_sec;
            tval_start.tv_usec = tval_now.tv_usec;

            // get the coordinates of where the leg should be (legs on left side of body will need reversed)
            //getDifference(&diff_coord, control.right_joy_x, -control.right_joy_y, elapsed);

            // flip inputs if direction is reversed
            if (direction > 0)
            {
                if (getWalkDiff(&diff_coord, &desired_coord, control.d_x, control.d_y, boundary, elapsed) && !firstloop)
                    flip = 1;
            }
            else
            {
                if (getWalkDiff(&diff_coord, &desired_coord, -control.d_x, -control.d_y, boundary, elapsed) && !firstloop)
                    flip = 1;
                printf("b\n");
                //diff_coord.x = -diff_coord.x;
                //diff_coord.y = -diff_coord.y;
            }

            firstloop = 0;
            //printf("flip: %d\n", flip);
            //printf("1 diff_coord= x: %f y: %f z: %f\n", diff_coord.x, diff_coord.y, diff_coord.z);

            // update desired_coord to the new desired point
        desired_coord.x = diff_coord.x;
        desired_coord.y = diff_coord.y;
            // !! desired_coord.z = diff_coord.z;

        //printf("2 desired_coord= x: %f y: %f z: %f\n", desired_coord.x, desired_coord.y, desired_coord.z);

            // flip coordinates for legs on opposite side

            local_coord.x = desired_coord.x;
            local_coord.y = desired_coord.y;


            // flip coordinates for legs that are touching the ground - this is unnessesary to flip coords, because they flip when changing direction...
            int on_ground = direction;

            if (!on_ground && !flip)
            {
                // lift legs that are not on the ground
                local_coord.z = STEP_Z;
            }
            else
                local_coord.z = START_Z;

            /*if (!on_ground)
            {
                local_coord.x = -local_coord.x;
                local_coord.y = -local_coord.y;
            }*/
            //printf("x: %f\ty: %f\n", desired_coord.x, desired_coord.y);
            // adjust y_coord to START_Y offset
        desired_coord.y += START_Y;
        local_coord.y += START_Y;
            usleep(100000);
            //get_angles_relative(leg_num, &desired_pos, &local_coord);

            //printf("actual ang: %f\t%f\t%f\n", actual_pos.angle1, actual_pos.angle2, actual_pos.angle3);
            //printf("desired: %f\t%f\t%f\n", desired_pos.angle1, desired_pos.angle2, desired_pos.angle3);

            // calculate how far we are from where we need to be
            //diff1 = fabsf(actual_pos.angle1 - desired_pos.angle1);
            //diff2 = fabsf(actual_pos.angle2 - desired_pos.angle2);
            //diff3 = fabsf(actual_pos.angle3 - desired_pos.angle3);
            //printf("diffs: %f\t%f\t%f\n", diff1, diff2, diff3);

            // travel CW or CCW rather than to a specific target angle
            /*if (coord.x != boundary)
             {
             if (actual_pos.angle1 - desired_pos.angle1 < 0)
             desired_pos.angle1 = MAX_ANGLE;
             else if (actual_pos.angle1 - desired_pos.angle1 > 0)
             desired_pos.angle1 = MIN_ANGLE;

             if (actual_pos.angle2 - desired_pos.angle2 < 0)
             desired_pos.angle2 = MAX_ANGLE;
             else if (actual_pos.angle2 - desired_pos.angle2 > 0)
             desired_pos.angle2 = MIN_ANGLE;

             if (actual_pos.angle3 - desired_pos.angle3 < 0)
             desired_pos.angle3 = MAX_ANGLE;
             else if (actual_pos.angle3 - desired_pos.angle3 > 0)
             desired_pos.angle3 = MIN_ANGLE;
             }*/

            // make sure that the speed values cannot be less than 1 (full speed if already near desired position = BAD)
            //speed1 = P*diff1 + 1;
            //speed2 = P*diff2 + 1;
            //speed3 = P*diff3 + 1;

            //printf("speeds = %f\t%f\t%f\n", speed1, speed2, speed3);

        // change direction
        if (flip)
        {
            // even when we want to change direction, we iterate the above for loop once more, to ensure all 6 legs are on the ground, and have reached the boundary
            change = 1;
        }
        if (change)
        {
            // !! TODO - maybe try telling/waiting for all legs to reach boundary here? - if last leg in for loop above was to reach boundary, others would fall short before changing direction
            //until_legs_stop();
            flip = 0;
            change = 0;
            firstloop = 1;
            direction = -direction;
            printf("stepping\n");
        }
    }
}

int main(int argc, char** argv)
{
    // open port for sub-serial bus
    char portName[] = "/dev/ttyUSB0";
    openPort(portName);

    printf("If program hangs here, make sure ALL 6 legs are plugged in, usb-serial bridge is powered ON, and controller is connected (via bluetooth)");

   	// loop until controller connects
   	while (openController(&control) == -1)
        getPresses(&control);

   	reposition_legs();

   	while(1)
   	{
        switch(mode)
        {
            case 0:
                control_body();
                break;
            case 1:
                if (option == 0)
                    walk();
                else
                    rotate();
                break;
            case 2:
                control_leg(option);
                break;
        }
        get_selection(&control);
   	}

    closeController();
    closePort();
}
