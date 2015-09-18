#include <stdio.h>
#include <math.h>
#include <signal.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>

#include "robotinterface.h"
#include "Configuration.h"
#include "socket.hpp"

#define BC_IP "192.168.0.255"
#define BC_PORT 2010

#define RC_IP "192.168.0.211"
#define RC_PORT 2011

char string[256];
int stringpointer = 0;
double position[6];
double readPosition[6];
double zero_vector[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
double position_vector[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
double speed_vector[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
double acceleration_vector[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
double current_vector[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
struct message_t message;
double orig_pos[6] = {0,0,0,0,0,0};
bool exit_var = false;
//double amplitude[6] = { 0, M_PI/16, -M_PI/8, -M_PI/4,   0, -M_PI/4   };
//double omega[6]     = { 0, M_PI*1.3, M_PI*1.1,    M_PI*0.8,     0, M_PI*0.7 };
double amplitude[6] = { 0, 0, 0, 0, 0, M_PI/8};
double omega[6]     = { 0, 0, 0, 0, M_PI, M_PI}; 
double commanded[10]; 
double current[10];

double
getTime (void)
{
    // Needs -lrt in Makefile
    struct timespec ts;
    if (clock_gettime (CLOCK_MONOTONIC, &ts) < 0)
    {
        perror ("clock_gettime");
        return -1;
    }
    return (double) ts.tv_sec + ((double) ts.tv_nsec) / 1e9;
}
static void
int_handler (int signal, siginfo_t * info, void *data)
{ 
    printf ("SIGINT\n");
    exit_var = true;
}

void 
catchSig ()
{
    struct sigaction act;
    act.sa_sigaction = int_handler;
    act.sa_flags = SA_RESTART | SA_SIGINFO;
    sigemptyset (&act.sa_mask);
    if (sigaction (SIGINT, &act, NULL) < 0)
    {
        perror ("sigaction");
        exit(0);
    }
}



int
main(int argc, char *argv[]) 
{
    int i, readSize;
    double controlTime, initTime;
    char buffer[1024];
    int fdUdpSend = connect_broadcast_udp_socket (BC_IP, BC_PORT);
    int fdUdpRecv = get_udp_server_socket (RC_PORT);

    // Enable catching of SIGINT
    catchSig ();
    printf("Example program start\n");
    message.text = string;
    printf("- Loading robot configuration\n");
    configuration_load();
    printf("- Resetting positions\n");
    for (i = 0; i < 6; i++) {
        position[i] = 0.0;
    }
    printf("- Opeing robotinterface (real robot)\n");
    robotinterface_open(0);
    for (i = 0; i < 100; i++) 
    {
        robotinterface_read_state_blocking();
        robotinterface_get_actual_position(position_vector);
        robotinterface_command_velocity(zero_vector);
        robotinterface_unlock_security_stop();
        robotinterface_send();
    }

    
    robotinterface_read_state_blocking();
    robotinterface_command_velocity(zero_vector);
    robotinterface_set_robot_running_mode();
    robotinterface_send();
    
    for (i = 0; i<6; i++) 
    {
        orig_pos[i] = position_vector[i];
    }
   

    printf("- Performing loop 2\n");

    initTime = getTime();
    for (i = 0; exit_var == false; i++) 
    {
        robotinterface_read_state_blocking();
        controlTime = getTime() - initTime;
        robotinterface_get_actual(position_vector, speed_vector);
        memcpy (buffer, position_vector,48);
        memcpy (buffer+48, speed_vector,48);

        write (fdUdpSend, buffer, 48*2);


        readSize = read (fdUdpRecv, buffer, 1024);
        if (readSize != 48*3)
        {
            printf("error in packet: size= %d\n", readSize);
            break;
        }
        memcpy (position_vector,buffer,48);
        memcpy (speed_vector,buffer+48,48);
        memcpy (acceleration_vector,buffer+48*2,48);
        //printf("%f %f %f\n",position_vector[5], speed_vector[5], acceleration_vector[5]);

        //printf("Got position %f for joint x ",position_vector[jointNum]);
        
        /*for (int jointNum = 0; jointNum<6; jointNum++)
        {
            position_vector[jointNum] = orig_pos[jointNum] - amplitude[jointNum] + amplitude[jointNum] * cos(controlTime * omega[jointNum]);
            speed_vector[jointNum] =  -(amplitude[jointNum] * omega[jointNum]) * sin(controlTime * omega[jointNum]);
            acceleration_vector[jointNum] =  -(amplitude[jointNum] * pow(omega[jointNum],2)) * cos(controlTime * omega[jointNum]);
        
            if ((i & 255) == 0) {
                printf("  %04x Setting pva command %f %f \n", i, position_vector[jointNum], speed_vector[jointNum]);
            }
        }*/

        //robotinterface_command_position_velocity_acceleration(orig_pos,zero_vector,zero_vector);
        robotinterface_command_position_velocity_acceleration(position_vector,
                                                              speed_vector,
                                                              acceleration_vector);
        
        robotinterface_send();


        // Flush messages to stdout
        while (robotinterface_get_message_count() > 0) 
        {
            printf("  %04x Handling message\n", i);
            stringpointer = 0;
            stringpointer += robotinterface_get_message(&message);
            printf("%s \n", message.text);
        } 

    }

    printf("- Speeding down\n");
    for (i = 0; i < 10; i++) 
    {
        robotinterface_read_state_blocking();
        //robotinterface_get_actual_position(position_vector);
        robotinterface_command_velocity(zero_vector);
        robotinterface_send();
    }
    printf("- Closing\n");
    robotinterface_close();
    printf("Done.\n");
    return 0;
}

