#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "ax12a.h"
#include "leg.h"

#define P1 1
#define P2 0.2
#define P3 0.2
#define STEP 1.0

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
           6. Walk forward\n");
           
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
        float x,y,z;
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
                coord.x=x-STEP;
            else if (c == '2')
                coord.x=x+STEP;
            else if (c == '3')
                coord.y=y-STEP;
            else if (c == '4')
                coord.y=y+STEP;
            else if (c == '5')
                coord.z=z-STEP;
            else if (c == '6')
                coord.z=z+STEP;
            
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
        float gamma, elapsed, speed, motor_speed, diff1, diff2, diff3;
        struct leg_status leg_stat;
        struct coordinate coord;
        struct position desired_pos, actual_pos;
        int leg_num = get_leg_num();
        coord.x = -20;
        coord.y = 20;
        coord.z = 0;
        speed = 5;  // cm/sec linear motion

        // "wind" leg back
        move_leg(leg_num, &coord);
        sleep(2);

        // set coordinates to a full forward stride position
        coord.x = 20;

        // get the angle of the servo closest to the body
        gamma = get_gamma(coord.x, coord.y);
        turnMotor(legs[leg_num][0], gamma, motor_speed);
        time_t start = time(NULL);
        
        while (1)
        {
            // find out where the servos are
            get_leg_status(leg_num, &leg_stat);
            actual_pos.angle1 = leg_stat.motors[1].position;
            actual_pos.angle2 = leg_stat.motors[2].position;
            actual_pos.angle3 = leg_stat.motors[3].position;

            // calculate where the servos should be
            time_t now = time(NULL);
            elapsed = now - start;
            coord.x = (speed / elapsed) - 20;

            if (coord.x > 20)
                coord.x = 20;

            get_angles(&desired_pos, &coord);
            
            // calculate how far we are from where we need to be
            diff1 = actual_pos.angle1 - desired_pos.angle1;
            diff2 = fabsf(actual_pos.angle2 - desired_pos.angle2);
            diff3 = fabsf(actual_pos.angle3 - desired_pos.angle3);
            
            // adjust accordingly
            motor_speed = motor_speed - P1 * diff1; // +/-??
            turnMotor(legs[leg_num][0], 20, motor_speed);
            turnMotor(legs[leg_num][1], desired_pos.angle2, P2*diff2);
            turnMotor(legs[leg_num][2], desired_pos.angle3, P3*diff3);
        }
    }
    else
    {
        printf("That is not a valid choice, program terminating.");
    }
    
    closePort();
    
    return 0;
}
