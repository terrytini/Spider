#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "ax12a.h"
#include "leg.h"
#include "input.h"

//#define P 0.7 //1 //.3    // this is the "P" value for motor#1 for our PID control loop
#define P2 1 //.3 //2.0 //3.0 //1.8
#define P3 1 //.3 //2.0 //3.0 //1.6
#define STEP 1.0
#define MIN_ANGLE 0     //min and max angles should be the full range for servos 2 and 3 (maybe seperated per motor if need be)
#define MAX_ANGLE 300   // TODO - measure/set these two (min and max angle) values
double P = 1;

// prompts user for, and returns desired leg number
int get_leg_num(void)
{
    char buffer[10];
    printf("enter a leg number (0-5) : ");
    scanf(" %s", buffer);
    return atoi(buffer);
}

// these are tests using/testing various methods
int main(void)
{
    char portName[] = "/dev/ttyUSB0";
    char buffer[20];
    int i, motorID, retval;
    float degree, speed;
    
    openPort(portName);
    
    printf("Please choose a method to test: \n\t\
           1. turnMotor\n\t\
           2. moveLeg\n\t\
           3. moveLeg (with number keys)\n\t\
           4. move leg in straight line along it's x axis\n\t\
           5. Move leg in straight line (forward to backward)\n\t\
           6. Walk forward (hard-coded loop)\n\t\
           7. Move leg along x-axis using a \"P\" control loop\n\t\
           8. Move leg along relative x-axis using control loop\n\t\
           9. Move leg along relative axis using control loop (controller as input)\n\t\
           10. Move leg along relative axis using control loop (controller as absolute input)\n");
    
    scanf(" %s", buffer);
    int choice = atoi(buffer);
    if(choice == 1)
    {
        while(1)
        {
            printf("Enter a motor ID number (0 - 253 or q to quit): ");
            scanf(" %s", buffer);
            
            if(buffer[0] == 'q')
            {
                break;
            }
            
            motorID = atoi(buffer);
            
            printf("Enter a goal position (0 - 300 or q to quit): ");
            scanf(" %s", buffer);
            
            if(buffer[0] == 'q')
            {
                break;
            }
            
            degree = atof(buffer);
            
            printf("Enter a speed (0 - ?? or q to quit): ");
            scanf(" %s", buffer);
            
            if(buffer[0] == 'q')
            {
                break;
            }
            
            speed = atof(buffer);
            
            retval = turnMotor(motorID, degree, speed);
            
            if(retval != 0)
            {
                printf("Error: %i\n", retval);
            }
            else
            {
                printf("Success!!!\n");
            }
        }
    }
    else if(choice == 2)
    {
        struct coordinate coord;
        
        while(1)
        {
            int leg_num = get_leg_num();
            printf("enter a point, x : ");
            scanf(" %s", buffer);
            coord.x = atof(buffer);
            printf("enter a point, y : ");
            scanf(" %s", buffer);
            coord.y = atof(buffer);
            printf("enter a point, z : ");
            scanf(" %s", buffer);
            coord.z = atof(buffer);
            
            move_leg(leg_num, &coord);
        }
        
        
    }
    else if(choice == 3)
    {
        struct position pos;
        int leg_num = get_leg_num();
        struct coordinate coord;
        coord.x=0;
        coord.y=20;
        coord.z=0;
        
        move_leg(leg_num, &coord);
        
        printf("Use the number keys to move the leg along the x, y, and z axes\n");
        
        while(1)
        {
            printf("\n1. decrease x\n2. increase x\n3. decrease y\n4. increase y\n5. decrease z\n6. increase z\n");
            char c = getchar();
            printf("%c\n", c);
            
            if (c == '1')
                coord.x=coord.x-STEP;
            else if (c == '2')
                coord.x=coord.x+STEP;
            else if (c == '3')
                coord.y=coord.y-STEP;
            else if (c == '4')
                coord.y=coord.y+STEP;
            else if (c == '5')
                coord.z=coord.z-STEP;
            else if (c == '6')
                coord.z=coord.z+STEP;
            
            move_leg(leg_num, &coord);
        }
    }
    else if(choice == 4)
    {
        float step = 1;
        struct coordinate coord;
        coord.x=0;
        coord.y=20;
        coord.z=0;
        int leg_num = get_leg_num();
        move_leg(leg_num, &coord);
        sleep(1);
        int boundary = 19;
        float time = 0.1;
        for (coord.x = 0; coord.x < boundary; coord.x+=step)
        {
            move_leg(leg_num, &coord);
            //sleep(time);
        }
        while(1)
        {
            for (coord.x = boundary; coord.x > -boundary; coord.x-=step)
            {
                move_leg(leg_num, &coord);
                //sleep(time);
            }
            for (coord.x = -boundary; coord.x < boundary; coord.x+=step)
            {
                move_leg(leg_num, &coord);
                //sleep(time);
            }
        }
    }
    else if(choice == 5)
    {
        float step = 1;
        float x0,y0,i;
        struct coordinate coord;
        coord.x=0;
        coord.y=20;
        coord.z=0;
        x0 = coord.x;
        y0 = coord.y;
        int leg_num = get_leg_num();
        move_leg(leg_num, &coord);
        usleep(3000000);
        int boundary = 10;
        float time = 700000;
        for (i = 0; i < boundary; i+=step)
        {
            if (leg_num == 0)
            {
                coord.x = x0+i/sqrt(2);
                coord.y = y0+i/sqrt(2);
            }
            else if (leg_num == 2)
            {
                coord.x = x0+i/sqrt(2);
                coord.y = y0-i/sqrt(2);
            }
            else if (leg_num == 3)
            {
                coord.x = x0-i/sqrt(2);
                coord.y = y0-i/sqrt(2);
            }
            else if (leg_num == 5)
            {
                coord.x = x0-i/sqrt(2);
                coord.y = y0+i/sqrt(2);
            }
            else
            {
                coord.x = x0 + i;
            }
            move_leg(leg_num, &coord);
            usleep(time);
        }
        while(1)
        {
            for (i = boundary; i > -boundary; i-=step)
            {
                if (leg_num == 0)
                {
                    coord.x = x0+i/sqrt(2);
                    coord.y = y0+i/sqrt(2);
                }
                else if (leg_num == 2)
                {
                    coord.x = x0+i/sqrt(2);
                    coord.y = y0-i/sqrt(2);
                }
                else if (leg_num == 3)
                {
                    coord.x = x0-i/sqrt(2);
                    coord.y = y0-i/sqrt(2);
                }
                else if (leg_num == 5)
                {
                    coord.x = x0-i/sqrt(2);
                    coord.y = y0+i/sqrt(2);
                }
                else
                {
                    coord.x = x0 + i;
                }
                move_leg(leg_num, &coord);
                usleep(time);
            }
            for (i = -boundary; i < boundary; i+=step)
            {
                if (leg_num == 0)
                {
                    coord.x = x0+i/sqrt(2);
                    coord.y = y0+i/sqrt(2);
                }
                else if (leg_num == 2)
                {
                    coord.x = x0+i/sqrt(2);
                    coord.y = y0-i/sqrt(2);
                }
                else if (leg_num == 3)
                {
                    coord.x = x0-i/sqrt(2);
                    coord.y = y0-i/sqrt(2);
                }
                else if (leg_num == 5)
                {
                    coord.x = x0-i/sqrt(2);
                    coord.y = y0+i/sqrt(2);
                }
                else
                {
                    coord.x = x0 + i;
                }
                
                
                move_leg(leg_num, &coord);
                usleep(time);
            }
        }
    }
    else if(choice == 7)
    {
        double gamma, speed, motor_speed, diff1, diff2, diff3, boundary, elapsed, speed1, speed2, speed3, angle2, angle3;
        struct leg_status leg_stat;
        struct coordinate coord;
        struct position desired_pos, actual_pos;
        struct timeval tval_start, tval_now, tval_elapsed;
        int leg_num = get_leg_num();
        boundary = 15;      // 1/2 of the full stride (bounds the movement along the x-axis)
        coord.x = -boundary;
        coord.y = 15;
        coord.z = 0;
        speed = 20;         // cm/sec linear motion
        motor_speed = 1;    // how fast motor1 will begin moving in the desired direction
        
        // "wind" leg back
        move_leg(leg_num, &coord);
        for (int i=0; i<3; i++)
            waitUntilStop(legs[leg_num][i]);
        
        while(1)
        {
            // set coordinates to a full forward stride position
            coord.x = boundary;
            
            // get the desired final angle of the servo closest to the body
            gamma = get_gamma(coord.x, coord.y) + 150;  //again, the angles are going off the angles from the diagram... this is needed to adjust the angle to the default angle of the servos.
            turnMotor(legs[leg_num][0], gamma, motor_speed);
            gettimeofday(&tval_start, NULL);
            coord.x = 0;
            while (coord.x != boundary)  // !! TODO -> should be changed to while (motor 1 is moving)
            {
                // find out where the servos are
                get_leg_status(leg_num, &leg_stat);
                actual_pos.angle1 = leg_stat.motors[0].position;
                actual_pos.angle2 = leg_stat.motors[1].position;
                actual_pos.angle3 = leg_stat.motors[2].position;
                printf("actual: %f\t%f\t%f\n", actual_pos.angle1, actual_pos.angle2, actual_pos.angle3);
                
                // calculate where the servos should be
                gettimeofday(&tval_now, NULL);
                elapsed = (tval_now.tv_sec - tval_start.tv_sec) + (tval_now.tv_usec - tval_start.tv_usec)/1000000.0;
                
                if (boundary > 0)
                {
                    coord.x = (speed * elapsed) - boundary;
                    
                    if (coord.x > boundary)
                        coord.x = boundary;
                }
                else
                {
                    coord.x = -boundary - (speed * elapsed);
                    
                    if (coord.x < boundary)
                        coord.x = boundary;
                }
                
                printf("elapsed : %f\tx : %f\n", elapsed, coord.x);
                
                get_angles(&desired_pos, &coord);
                desired_pos.angle1 = desired_pos.angle1 + 150;     // get_angles() currently only returns the angles from the inverse kinematics diagrams we found online, these values need adjusted to the default angles for our servos (I should probably just have resolved this in get angles...)
                desired_pos.angle2 = desired_pos.angle2 + 60;
                desired_pos.angle3 = 360 - desired_pos.angle3 - 138;
                printf("desired: %f\t%f\t%f\n", desired_pos.angle1, desired_pos.angle2, desired_pos.angle3);
                
                // calculate how far we are from where we need to be
                diff1 = actual_pos.angle1 - desired_pos.angle1;
                if (boundary < 0)
                    diff1 = -diff1;
                diff2 = fabsf(actual_pos.angle2 - desired_pos.angle2);
                diff3 = fabsf(actual_pos.angle3 - desired_pos.angle3);
                printf("diffs: %f\t%f\t%f\n", diff1, diff2, diff3);
                
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
                if (-P*diff1 < 1)
                    speed1 = 1;
                else
                    speed1 = -P*diff1;
                
                if (P2*diff2 < 1)
                    speed2 = 1;
                else
                    speed2 = P2*diff2;
                
                if (P3*diff3 < 1)
                    speed3 = 1;
                else
                    speed3 = P3*diff3;
                
                printf("speeds = %f\t%f\t%f\n", speed1, speed2, speed3);
                
                // adjust accordingly
                turnMotor(legs[leg_num][0], desired_pos.angle1, speed1);
                turnMotor(legs[leg_num][1], desired_pos.angle2, speed2);
                turnMotor(legs[leg_num][2], desired_pos.angle3, speed3);
            }
            //speed1 = motor_speed;    // !!?? should we reset the motor speed? or leave it the same?
            boundary = -boundary;   // switch direction
        }
    }
    else if(choice == 8)
    {
        double gamma, speed, motor_speed, diff1, diff2, diff3, boundary, elapsed, speed1, speed2, speed3, angle2, angle3;
        struct leg_status leg_stat;
        struct coordinate coord;
        struct position desired_pos, actual_pos;
        struct timeval tval_start, tval_now, tval_elapsed;
        int leg_num = get_leg_num();
        boundary = 10;      // 1/2 of the full stride (bounds the movement along the x-axis)
        coord.x = -boundary;
        coord.y = 15;
        coord.z = 0;
        speed = 20;         // cm/sec linear motion
        motor_speed = 1;    // how fast motor1 will begin moving in the desired direction
        
        // "wind" leg back
        move_leg(leg_num, &coord);
        for (int i=0; i<3; i++)
            waitUntilStop(legs[leg_num][i]);
        
        while(1)
        {
            // set coordinates to a full forward stride position
            coord.x = boundary;
            
            // get the desired final angle of the servo closest to the body
            gamma = get_gamma(coord.x, coord.y) + 150;  //again, the angles are going off the angles from the diagram... this is needed to adjust the angle to the default angle of the servos.
            turnMotor(legs[leg_num][0], gamma, motor_speed);
            gettimeofday(&tval_start, NULL);
            coord.x = 0;
            while (coord.x != boundary)//(sqrt(sq(coord.x)+sq(coord.y-15)) != boundary) !! ?? ROUND OFF ERROR
            {
                // find out where the servos are
                get_leg_status(leg_num, &leg_stat);
                actual_pos.angle1 = leg_stat.motors[0].position;
                actual_pos.angle2 = leg_stat.motors[1].position;
                actual_pos.angle3 = leg_stat.motors[2].position;
                printf("actual: %f\t%f\t%f\n", actual_pos.angle1, actual_pos.angle2, actual_pos.angle3);
                
                // calculate where the servos should be
                gettimeofday(&tval_now, NULL);
                elapsed = (tval_now.tv_sec - tval_start.tv_sec) + (tval_now.tv_usec - tval_start.tv_usec)/1000000.0;
                
                if (boundary > 0)
                {
                    coord.x = (speed * elapsed) - boundary;
                    
                    if (coord.x > boundary)
                        coord.x = boundary;
                }
                else
                {
                    coord.x = -boundary - (speed * elapsed);
                    
                    if (coord.x < boundary)
                        coord.x = boundary;
                }
                
                printf("elapsed : %f\tx : %f\n", elapsed, coord.x);
                
                get_angles_relative(leg_num, &desired_pos, &coord);
                
                printf("desired: %f\t%f\t%f\n", desired_pos.angle1, desired_pos.angle2, desired_pos.angle3);
                
                // calculate how far we are from where we need to be
                diff1 = actual_pos.angle1 - desired_pos.angle1;
                if (boundary < 0)
                    diff1 = -diff1;
                diff2 = fabsf(actual_pos.angle2 - desired_pos.angle2);
                diff3 = fabsf(actual_pos.angle3 - desired_pos.angle3);
                printf("diffs: %f\t%f\t%f\n", diff1, diff2, diff3);
                
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
                if (-P*diff1 < 1)
                    speed1 = 1;
                else
                    speed1 = -P*diff1;
                
                if (P2*diff2 < 1)
                    speed2 = 1;
                else
                    speed2 = P2*diff2;
                
                if (P3*diff3 < 1)
                    speed3 = 1;
                else
                    speed3 = P3*diff3;
                
                printf("speeds = %f\t%f\t%f\n", speed1, speed2, speed3);
                
                // adjust accordingly
                turnMotor(legs[leg_num][0], desired_pos.angle1, speed1);
                turnMotor(legs[leg_num][1], desired_pos.angle2, speed2);
                turnMotor(legs[leg_num][2], desired_pos.angle3, speed3);
            }
            boundary = -boundary;   // switch direction
        }
    }
    else if(choice == 9)
    {
        double gamma, motor_speed, diff1, diff2, diff3, boundary, elapsed, speed1, speed2, speed3, angle2, angle3;
        struct leg_status leg_stat;
        struct coordinate desired_coord, actual_coord, diff_coord;
        struct position desired_pos, actual_pos;
        struct timeval tval_start, tval_now, tval_elapsed;
        struct controller control;
        int leg_num = get_leg_num();
        int direction = 1;
        boundary = 8;      // 1/2 of the full stride (bounds the movement along the x-axis)
        desired_coord.x = 0;//-boundary;
        desired_coord.y = 15;
        desired_coord.z = 0;
        motor_speed = 1;    // how fast motor1 will begin moving in the desired direction
        
        openController(&control);
        
        // position leg
        move_leg_relative(leg_num, &desired_coord);
        for (int i=0; i<3; i++)
            waitUntilStop(legs[leg_num][i]);
        
        //while(1)
        {
            // set coordinates to a full forward stride position
            //desired_coord.x = boundary;
            // get the desired final angle of the servo closest to the body
            //gamma = get_gamma(desired_coord.x, desired_coord.y) + 150;  //again, the angles are going off the angles from the diagram... this is needed to adjust the angle to the default angle of the servos.
            //turnMotor(legs[leg_num][0], gamma, motor_speed);
            gettimeofday(&tval_start, NULL);
            //desired_coord.x = 0;
            //!! should be START_Y below - not 15
            while (sqrt(sq(desired_coord.x)+sq(desired_coord.y-15)) < boundary && !control.ps_button)  //!! either != boundary (and TODO - set boundary to boundary if outside of it) OR < boundary
            {
                // find out where the servos are
                get_leg_status(leg_num, &leg_stat);
                
                actual_pos.angle1 = leg_stat.motors[0].position;    // !! TODO - we may have to change this to do the adjustment between reading from each motor in order to minimize delay
                actual_pos.angle2 = leg_stat.motors[1].position;
                actual_pos.angle3 = leg_stat.motors[2].position;
                
                //get the coordinates of the tip of the leg
                get_position_relative(leg_num, &actual_coord, &actual_pos);
                printf("actual coord: x:%f\ty:%f\tz:%f\n", actual_coord.x, actual_coord.y, actual_coord.z);
                
                // calculate where the servos should be
                
                // get stick inputs
                getPresses(&control);   //this needs to be in real time, but there is currently some delay introduced in that every read just gives the NEXT read, but we want the CURRENT read
                
                gettimeofday(&tval_now, NULL);
                elapsed = (tval_now.tv_sec - tval_start.tv_sec) + (tval_now.tv_usec - tval_start.tv_usec)/1000000.0;
                tval_start.tv_sec = tval_now.tv_sec;
                tval_start.tv_usec = tval_now.tv_usec;
                
                //instead of getting difference here, just get absolute position (relative to joystick inputs) - no need for timer
                getDifference(&diff_coord, control.left_joy_x, control.left_joy_y, elapsed);//(speed * elapsed) - boundary;
                printf("diff_coord= x: %f y: %f z: %f\n", diff_coord.x, diff_coord.y, diff_coord.z);
                
                desired_coord.x = desired_coord.x + diff_coord.x;
                desired_coord.y = desired_coord.y + diff_coord.y;
                desired_coord.z = desired_coord.z + diff_coord.z;
                
                get_angles_relative(leg_num, &desired_pos, &desired_coord);
                
                printf("actual ang: %f\t%f\t%f\n", actual_pos.angle1, actual_pos.angle2, actual_pos.angle3);
                printf("desired: %f\t%f\t%f\n", desired_pos.angle1, desired_pos.angle2, desired_pos.angle3);
                
                // calculate how far we are from where we need to be
                diff1 = actual_pos.angle1 - desired_pos.angle1; //!! can take abs val here?
                if (boundary < 0)
                    diff1 = -diff1;
                diff2 = fabsf(actual_pos.angle2 - desired_pos.angle2);
                diff3 = fabsf(actual_pos.angle3 - desired_pos.angle3);
                printf("diffs: %f\t%f\t%f\n", diff1, diff2, diff3);
                
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
                if (-P*diff1 < 1)
                    speed1 = 1;
                else
                    speed1 = -P*diff1;
                
                if (P2*diff2 < 1)
                    speed2 = 1;
                else
                    speed2 = P2*diff2;
                
                if (P3*diff3 < 1)
                    speed3 = 1;
                else
                    speed3 = P3*diff3;
                
                printf("speeds = %f\t%f\t%f\n", speed1, speed2, speed3);
                
                // adjust accordingly
                turnMotor(legs[leg_num][0], desired_pos.angle1, speed1);
                turnMotor(legs[leg_num][1], desired_pos.angle2, speed2);
                turnMotor(legs[leg_num][2], desired_pos.angle3, speed3);
            }
            direction = -direction;
        }
    }
    // absolute control - we will use this loop to control a single leg at a time with the controller
    // TODO - tune P value
    // TODO - myabe we should not use relative methods for this?
    else if(choice == 10)
    {
        double P = 0.95; // make a constant!!
        double boundary = 10;           // 1/2 of the full stride (radius of leg movement in x-y plane)
        double diff1, diff2, diff3;     // abs vals of differences between the current and desired angles for servos 1, 2, and 3 respectively
        double speed1, speed2, speed3;   // speed values to turn each motor at
        double z_input;                 // the controller input value for the z_axis
        struct leg_status leg_stat;     // holds the statuses (speeds and positions) for each motor on a leg
        struct coordinate desired_coord, actual_coord, diff_coord;
        struct position desired_pos, actual_pos;
        struct controller control;      // holds the button presses (must call get_presses() on it to update)
        
        desired_coord.x = 0;    // The starting coordinates of the legs
        desired_coord.y = 17;// !! 15;  // TODO - change to START_Y or RELAX_Y...
        desired_coord.z = 0;
        
        openController(&control);
        
        int leg_num = get_leg_num();
        
        // position leg to its starting (relaxed) position
        move_leg_relative(leg_num, &desired_coord);
        for (int i=0; i<3; i++)
            waitUntilStop(legs[leg_num][i]);
        
        while (!control.ps_button)//(sqrt(sq(desired_coord.x)+sq(desired_coord.y-15)) < boundary && !control.ps_button)  //!! either != boundary (and TODO - set boundary to boundary if outside of it) OR < boundary
        {
            // find out where the servos are
            get_leg_status(leg_num, &leg_stat);
            
            actual_pos.angle1 = leg_stat.motors[0].position;    // !! we may have to change this to do the adjustment between reading from each motor in order to minimize delay
            actual_pos.angle2 = leg_stat.motors[1].position;
            actual_pos.angle3 = leg_stat.motors[2].position;
            
            //get the coordinates of the tip of the leg
            get_position_relative(leg_num, &actual_coord, &actual_pos);
            printf("actual coord: x:%f\ty:%f\tz:%f\n", actual_coord.x, actual_coord.y, actual_coord.z);
            
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
            
            desired_coord.y += 17; // TODO - shoud be START_Y instead of hard coded number
            
            get_angles_relative(leg_num, &desired_pos, &desired_coord);
            
            printf("actual ang: %f\t%f\t%f\n", actual_pos.angle1, actual_pos.angle2, actual_pos.angle3);
            printf("desired: %f\t%f\t%f\n", desired_pos.angle1, desired_pos.angle2, desired_pos.angle3);
            
            // calculate how far we are from where we need to be
            diff1 = fabsf(actual_pos.angle1 - desired_pos.angle1);
            diff2 = fabsf(actual_pos.angle2 - desired_pos.angle2);
            diff3 = fabsf(actual_pos.angle3 - desired_pos.angle3);
            printf("diffs: %f\t%f\t%f\n", diff1, diff2, diff3);
            
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
            
            printf("speeds = %f\t%f\t%f\n", speed1, speed2, speed3);
            
            // adjust accordingly
            turnMotor(legs[leg_num][0], desired_pos.angle1, speed1);
            turnMotor(legs[leg_num][1], desired_pos.angle2, speed2);
            turnMotor(legs[leg_num][2], desired_pos.angle3, speed3);
        }
    }
    // absolute control makes the legs move to the exact position they are told - as opposed to giving them a direction to move in and a speed to move in that direction (as we will need to do for walking/rotating)
    // absolute control of body (forward/back, left/right, up/down)
    // this loop allows for absolute control of the coordinates of the center of the body using the controller
    else if(choice == 11)
    {
        double P = 0.55;
        double boundary = 10;           // 1/2 of the full stride (radius of leg movement in x-y plane)
        double diff1, diff2, diff3;     // abs vals of differences between the current and desired angles for servos 1, 2, and 3 respectively
        double speed1, speed2, speed3;  // speed values to turn each servo motor at
        double z_input;                 // the controller input value for the z_axis
        double z_up_bound = 8, z_down_bound = 6;// the maximum movement that the body can make up or down from zero in the z-axis
        struct leg_status leg_stat;     // holds the statuses (speeds and positions) for each motor on a leg
        struct coordinate desired_coord, actual_coord, diff_coord;
        struct position desired_pos, actual_pos;
        struct controller control;      // holds the button presses (must call get_presses() on it to update)
        
        double elapsed;                 // amount of time that elapsed since last update
        struct timeval tval_start, tval_now;
        int direction = 1;  // TODO - remove this variable from here (not yet... to be used in the walking loop)
        desired_coord.x = 0;    // The starting coordinates of the legs !! TODO - these should be set according to some constants
        desired_coord.y = 17;   //15;
        desired_coord.z = 0;
        
        printf("If program hangs here, make sure ALL 6 legs are plugged in, usb-serial bridge is powered ON, and controller is connected (via bluetooth)");
        
        openController(&control);
        
        // position legs to their starting (relaxed) position
        for (int i=0; i<6; i++)
        {
            move_leg_relative(i, &desired_coord);
            for (int j=0; j<3; j++)
                waitUntilStop(legs[i][j]);
        }
        
        // control loop
        while(!control.ps_button)
        {
            
            gettimeofday(&tval_start, NULL);
            
            while (!control.ps_button)//(sqrt(sq(desired_coord.x)+sq(desired_coord.y-15)) < boundary && !control.ps_button)  //!! either != boundary (and TODO - set boundary to boundary if outside of it) OR < boundary
            {
                // maybe we should get the controller input on each loop iteration below??
                //getPresses(&control);

                // find out where the servos are
                for (int leg_num = 0; leg_num<6; leg_num++)
                {
                    get_leg_status(leg_num, &leg_stat);
                    
                    actual_pos.angle1 = leg_stat.motors[0].position;    // !! TODO - we may have to change this to do the adjustment between reading from each motor in order to minimize delay
                    actual_pos.angle2 = leg_stat.motors[1].position;
                    actual_pos.angle3 = leg_stat.motors[2].position;
                    
                    //get the coordinates of the tip of the leg
                    get_position_relative(leg_num, &actual_coord, &actual_pos);
                    printf("actual coord: x:%f\ty:%f\tz:%f\n", actual_coord.x, actual_coord.y, actual_coord.z);
                    
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
                    desired_coord.x = desired_coord.x * sq(0.95 - fabsf(z_input));   // !! adjust what we are subtracting the abs val from here to allow a little movement at extreme z-coords
                    desired_coord.y = desired_coord.y * sq(0.95 - fabsf(z_input));

                    if (z_input < 0)
                        desired_coord.z *= z_up_bound;
                    else
                        desired_coord.z *= z_down_bound;

                    if (leg_num/3 < 1)
                    {
                        // if the leg is on the left side of the body, flip the inputs (x and y axes are rotated 180 degrees)
                        desired_coord.x = -desired_coord.x;
                        desired_coord.y = -desired_coord.y;
                    }
                    
                    desired_coord.y += 17; // TODO - shoud be START_Y instead of hard coded number
                    
                    //desired_coord.x = desired_coord.x + diff_coord.x;
                    //desired_coord.y = desired_coord.y + diff_coord.y;
                    //desired_coord.z = desired_coord.z + diff_coord.z;
                    
                    get_angles_relative(leg_num, &desired_pos, &desired_coord);
                    
                    printf("actual ang: %f\t%f\t%f\n", actual_pos.angle1, actual_pos.angle2, actual_pos.angle3);
                    printf("desired: %f\t%f\t%f\n", desired_pos.angle1, desired_pos.angle2, desired_pos.angle3);
                    
                    // calculate how far we are from where we need to be
                    diff1 = fabsf(actual_pos.angle1 - desired_pos.angle1);
                    diff2 = fabsf(actual_pos.angle2 - desired_pos.angle2);
                    diff3 = fabsf(actual_pos.angle3 - desired_pos.angle3);
                    printf("diffs: %f\t%f\t%f\n", diff1, diff2, diff3);
                    
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

                    printf("speeds = %f\t%f\t%f\n", speed1, speed2, speed3);
                    
                    // adjust accordingly
                    turnMotor(legs[leg_num][0], desired_pos.angle1, speed1);
                    turnMotor(legs[leg_num][1], desired_pos.angle2, speed2);
                    turnMotor(legs[leg_num][2], desired_pos.angle3, speed3);
                }
            }
            direction = -direction;
        }
    }
    else if (choice == 20)
    {
        struct coordinate actual_coord, desired_coord;
        struct position actual_pos, desired_pos;
        
        desired_coord.x = -2.3;
        desired_coord.y = 14.1;
        desired_coord.z = 1.6;
        
        get_angles_relative(0, &desired_pos, &desired_coord);
        get_position_relative(0, &actual_coord, &desired_pos);
        
        //actual coord should now be (about) equal to desired coord
        printf("actual x:%f y:%f z:%f\n", actual_coord.x, actual_coord.y, actual_coord.z);
        printf("desire x:%f y:%f z:%f\n", desired_coord.x, desired_coord.y, desired_coord.z);
    }
    else if (choice == 21)
    {
        struct controller control;
        openController(&control);
        struct coordinate coord;
        while(1)
        {
            getPresses(&control);
            getAbsolute(&coord, control.left_joy_x, control.left_joy_y, control.d_y);//(speed * elapsed) - boundary;
            printf("x: %f\ty: %f\tz: %f\n", coord.x, coord.y, coord.z);
        }
    }
    else if(choice == 99)
    {
        float position, speed;
        struct position pos;
        int leg_num = get_leg_num();
        struct coordinate coord;
        coord.x=0;
        coord.y=20;
        coord.z=0;
        
        move_leg(leg_num, &coord);
        sleep(2);
        getPresentPositionSpeed(legs[leg_num][0], &position, &speed);
        printf("position: %f\tspeed: %f\n", position, speed);
        
        printf("Use the number keys to move the leg along the x, y, and z axes\n");
        
        while(1)
        {
            //printf("\n1. decrease x\n2. increase x\n3. decrease y\n4. increase y\n5. decrease z\n6. increase z\n");
            char c = getchar();
            printf("%c\n", c);
            
            if (c == '1')
                coord.x=coord.x-STEP;
            else if (c == '2')
                coord.x=coord.x+STEP;
            else if (c == '3')
                coord.y=coord.y-STEP;
            else if (c == '4')
                coord.y=coord.y+STEP;
            else if (c == '5')
                coord.z=coord.z-STEP;
            else if (c == '6')
                coord.z=coord.z+STEP;
            
            move_leg(leg_num, &coord);
            //sleep(1);
            
            //get_motor_status(int id, struct motor_status* motor_stat)
            float position, speed;
            getPresentPositionSpeed(legs[leg_num][0], &position, &speed);
            
            printf("position: %f\tspeed: %f\n", position, speed);
        }
    }
    else
    {
        printf("That is not a valid choice, program terminating.");
    }
    
    closePort();
    
    return 0;
}
