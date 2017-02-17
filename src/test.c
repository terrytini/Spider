#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "ax12a.h"
#include "leg.h"

#define P 1.3    // this is the "P" value for motor#1 for our PID control loop
#define P2 3.0//1.8
#define P3 3.0//1.6
#define STEP 1.0
#define MIN_ANGLE 0     //min and max angles should be the full range for servos 2 and 3 (maybe seperated if need be)
#define MAX_ANGLE 300

int get_leg_num(void)
{
    char buffer[10];
    printf("enter a leg number (0-5) : ");
    scanf(" %s", buffer);
    return atoi(buffer);
}

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
           6. Walk forward\n\t\
           7. Move leg along x-axis using a (primitive) \"P\" control loop\n\t\
           8. Move leg along x-axis using \"P\" control loop (bounded motor 2 and 3)\n\t\
           9. Move leg along x-axis using \"P\" control loop (I think this is the one we will end up using)\n\t");
    
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
    else if(choice == 6)
    {
        float step = 1;
        float x0,y0,i;
        int boundary = 9;   //  this is how far the leg will travel in either direction along the x axis from x = 0 (total stride = 2 * boundary)
        float time = 1000;
        struct coordinate coord1, coord2;
        coord1.x=0;
        coord1.y=20;
        coord1.z=0;
        coord2.x=0;
        coord2.y=20;
        coord2.z=3;     // how high the leg is picked up during a step
        
        //int leg_num = get_leg_num();
        for(int leg_num = 0; leg_num < 6; leg_num++)
            move_leg(leg_num,&coord1);
        
        usleep(2500000);
        
        for (i = 0; i < boundary; i+=step)
        {
            for(int leg_num = 0; leg_num < 6; leg_num+=2)
            {
                move_leg_relative(leg_num, &coord1, i);
            }
            for(int leg_num = 1; leg_num < 6; leg_num+=2)
            {
                move_leg_relative(leg_num, &coord2, i);
            }
            //usleep(time);
        }
        
        while(1)
        {
            for (i = boundary; i > -boundary; i-=step)
            {
                for(int leg_num = 0; leg_num < 6; leg_num+=2)
                {
                    move_leg_relative(leg_num, &coord2, i);
                }
                for(int leg_num = 1; leg_num < 6; leg_num+=2)
                {
                    move_leg_relative(leg_num, &coord1, i);
                }
                //usleep(time);
            }
            
            for (i = -boundary; i < boundary; i+=step)
            {
                for(int leg_num = 0; leg_num < 6; leg_num+=2)
                {
                    move_leg_relative(leg_num, &coord1, i);
                }
                for(int leg_num = 1; leg_num < 6; leg_num+=2)
                {
                    move_leg_relative(leg_num, &coord2, i);
                }
                //usleep(time);
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
        speed = 25;         // cm/sec linear motion
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
            //time_t start = time(NULL);    //only accurate to the second...
            gettimeofday(&tval_start, NULL);
            coord.x = 0;
            while (isMoving(legs[leg_num][0]))//(coord.x != boundary)  // !! TODO -> should be changed to while (motor 1 is moving)
            {
                // find out where the servos are
                get_leg_status(leg_num, &leg_stat);
                actual_pos.angle1 = leg_stat.motors[0].position;
                actual_pos.angle2 = leg_stat.motors[1].position;
                actual_pos.angle3 = leg_stat.motors[2].position;
                printf("actual: %f\t%f\t%f\n", actual_pos.angle1, actual_pos.angle2, actual_pos.angle3);
                
                // calculate where the servos should be
                gettimeofday(&tval_now, NULL);
                //timersub(&tval_now, &tval_start, &tval_elapsed);
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
                
                // make sure that the speed values cannot be zero (full speed if already near desired position = BAD)
                if (-P*diff1 < 1)
                    speed1 = 1;
                else
                    speed1 = -P*diff1;
                
                printf("speeds = %f\t%f\t%f\n", speed1, speed2, speed3);
                
                // adjust accordingly
                turnMotor(legs[leg_num][0], gamma, speed1);
                turnMotor(legs[leg_num][1], desired_pos.angle2, 0);
                turnMotor(legs[leg_num][2], desired_pos.angle3, 0);
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
        boundary = 15;      // 1/2 of the full stride (bounds the movement along the x-axis)
        coord.x = -boundary;
        coord.y = 15;
        coord.z = 0;
        speed = 25;         // cm/sec linear motion
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
                    if (actual_pos.angle2 - desired_pos.angle2 < 0)
                        angle2 = MAX_ANGLE;
                    else if (actual_pos.angle2 - desired_pos.angle2 > 0)
                        angle2 = MIN_ANGLE;
                    
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
                turnMotor(legs[leg_num][0], gamma, speed1);
                turnMotor(legs[leg_num][1], desired_pos.angle2, speed2);
                turnMotor(legs[leg_num][2], desired_pos.angle3, speed3);
            }
            //speed1 = motor_speed;    // !!?? should we reset the motor speed? or leave it the same?
            boundary = -boundary;   // switch direction
        }
    }
    else if(choice == 9)
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
    else if(choice == 10)
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
        speed = 30;         // cm/sec linear motion
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
            //speed1 = motor_speed;    // !!?? should we reset the motor speed? or leave it the same?
            boundary = -boundary;   // switch direction
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
