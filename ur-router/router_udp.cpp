/*
This proram provides a TCP network interface to some functions
of the UR_COMM_API. It has to run on the Universal Robots
controller box.

This program is very experimental and the use of it can lead to 
damage of personal and hardware. DO NOT USE IT UNDER ANY 
CIRCUMSTANCES!!! It is just for demonstration purposes to show
how to access the UR_COMM_API.

The code, the protocol, the logic, etc., may be subject to change 
without any further notice.

Copyright: NTNU, SINTEF 2012
Authors: Johannes Schrimpf and Morten Lind
*/

/*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <cstring>

using namespace std;

#include <robotinterface.h>
#include <Configuration.h>
#include <common/microprocessor_commands.h>
#include <common/microprocessor_definitions.h>

#include "socket.hpp"
#include "startup_utils.hpp"
#include "base_utils.hpp"
#include "interrupt_utils.hpp"

MessageHandler msg_handler;

#define BC_IP "192.168.1.0"
#define BC_PORT 2010
#define LOG_IP "192.168.0.101"
#define LOG_PORT 2012
#define RC_IP "192.168.0.216"
#define RC_PORT 2011

#define MAX_PACKET_MISS 3

void stop_robot(){
    robotinterface_command_velocity(zero_vector);
    robotinterface_send();
    exit_flag = true;
}

int main(int argc, char *argv[]) {
    unsigned long cycle_number = 0;
    short j, read_size;
    double control_time, init_time;
    
    setup_sigint(); // Enable catching of SIGINT
    cout << "=== Starting routing program ===" << endl;
    cout << "- Loading robot configuration" << endl;
    configuration_load();
    cout << "Setting tool payload: " << tcp_payload << endl;
    robotinterface_set_tcp_payload(tcp_payload);
    //configuration_print();
    //extern struct Configuration conf;
    cout << "Gravity (Tool mass) : " << conf.gravity[0] << ", " << conf.gravity[1] << ", " << conf.gravity[2] << " (" << conf.mass[5] << ")"<< endl;
    
    
    // printf("- Resetting positions\n");
    // for (j = 0; j < 6; j++) {
    //     position[j] = 0.0;
    // }
    
    open_interface();
    power_up();
    set_wrench();
    initialize();
    settle();
    
    cout << "=== Starting main loop ===" << endl;
    init_time = get_time();
    // Main Loop
    char recv_buffer[1024];
    int fd_udp_send = connect_broadcast_udp_socket (BC_IP, BC_PORT);
    int fd_udp_recv = get_udp_server_socket (RC_PORT);
    // int fdLogSock = connect_broadcast_udp_socket (LOG_IP, LOG_PORT);
    //int recv_counter;
    unsigned int packet_miss_counter = 0;

    for (cycle_number = 0; exit_flag == false; cycle_number++) {
        robotinterface_read_state_blocking();
        control_time = get_time() - init_time;
        /// Pack and send a servo status
        servo_packet.header.protocol_id = 0x00;
        servo_packet.header.cycle_number = cycle_number;
        robotinterface_get_actual(servo_packet.position, servo_packet.velocity);
        write (fd_udp_send, (void *) &servo_packet, sizeof(ServoPacket));
        /// Try to receive a command packet within 7ms
        fd_set read_fds;
        struct timeval timeout;
        FD_ZERO(&read_fds);
        FD_SET(fd_udp_recv, &read_fds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 3000;
        if (select(fd_udp_recv+1, &read_fds, NULL, NULL, &timeout) < 0) {
            cout << "Error in select for receiving. Shutting down." << endl;
            stop_robot();
            break;
        }
        /// Test and handle, if data is waiting
        if (FD_ISSET(fd_udp_recv, &read_fds)) {
            /// Got a valid packet in time. Clear the missed package counter
            packet_miss_counter = 0;
            /// Get data from external controller
            read_size = read (fd_udp_recv, recv_buffer, 1024);
            /// Guard against wrong packet sizes
            if (read_size != sizeof(PPacket) && read_size != sizeof(PVAPacket)){
                cout << "error in packet: size=" << read_size <<endl;
                stop_robot();
                break;
            }
            /// Read into a PVAPacket
            memcpy((void *) &pva_packet, (void *) recv_buffer, read_size);
            /// Warn on wrong cycle number
            if (pva_packet.header.cycle_number != cycle_number) {
                cout << "Warning!!!! Sequence error" << endl;
            }
            /// Extrapolate velocities and accelerations, if a pure position packet was received
            if (read_size == sizeof(PPacket)){
                for(j=0; j<6;j++){
                    pva_packet.velocity[j] = (pva_packet.position[j] - last_pva_packet.position[j])/0.008; 
                    pva_packet.acceleration[j] = (pva_packet.velocity[j] - last_pva_packet.velocity[j])/0.008; 
                }
            }
            /// Send the new pva command
            robotinterface_command_position_velocity_acceleration(pva_packet.position,
                                                                  pva_packet.velocity,
                                                                  pva_packet.acceleration);
        } else { 
            /// The select must have timed out, and no packet is received.             
            /// Test and increase the missed packet counter
            if(packet_miss_counter >= MAX_PACKET_MISS){
                /// Must drop out
		if (cycle_number%125 == 0){
		    cout << "!!! Resting due to too many packet misses !!!" << endl;
		}
                robotinterface_command_velocity(zero_vector);
               //stop_robot();
            } else { 
                packet_miss_counter++;
                cout << "!!! Warning !!! : Missed packet deadline (count " << packet_miss_counter << "). Continuing with strategy: ";
                /// Strategy: Maintain current velocity, with zero acceleration
                cout << "Maintained velocity."<<endl;
                for(j=0; j<6; j++){
                    pva_packet.position[j] += 0.008 * pva_packet.velocity[j];
                    pva_packet.acceleration[j] = 0.0;
                }
                /// Strategy: Just command zero velocity.
                //cout << "Maintained velocity."<<endl;
                //robotinterface_command_velocity(zero_vector);
                
                /// Send the synthetic command.
                robotinterface_command_position_velocity_acceleration(pva_packet.position,
                                                                      pva_packet.velocity,
                                                                      pva_packet.acceleration);
            }
        }
        
         // Commit changes
        robotinterface_send();
        /// Save pva data for extrapolation
        memcpy((void *) &last_pva_packet, (void *) &pva_packet, sizeof(PVAPacket));
        // Flush messages to stdout
        msg_handler.flush();
    }
    
    cout << "- Speeding down" <<endl;
    for (j = 0; j < 10; j++) {
        robotinterface_read_state_blocking();
        robotinterface_command_velocity(zero_vector);
        robotinterface_send();
    }
    cout << "- Closing" << endl;
    robotinterface_close();
    cout << "Done." << endl;
    return 0;
}

