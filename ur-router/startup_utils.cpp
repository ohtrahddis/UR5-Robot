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

#include <cstdlib>
#include <iostream>
#include <cstring>

using namespace std;

#include <robotinterface.h>
#include <common/microprocessor_commands.h>
#include <common/microprocessor_definitions.h>

#include "startup_utils.hpp"
#include "base_utils.hpp"

bool initialized = false;

void open_interface(){
    int retryCounter;
    cout << "Opening robotinterface (real robot)" << endl;
    cout << "Mode: " << robotinterface_get_robot_mode() << endl;
    /* open robotinterface */
    if (robotinterface_open(0) == FALSE) {
        cout << "robotinterface_open() failed" << endl;
        exit(EXIT_FAILURE);
    }
    
    retryCounter = 100;
    while (retryCounter>0 && !robotinterface_is_robot_connected()) {
        robotinterface_read_state_blocking();
        robotinterface_command_velocity(zero_vector);
        robotinterface_send();
        retryCounter--;
    }

    if (!robotinterface_is_robot_connected()) {
        robotinterface_close();
        cout << "Real robot not connected" << endl;
        exit(EXIT_FAILURE);
    }
    cout << "Enabling robot after securtity stop" << endl;
    while (robotinterface_get_robot_mode() == 4){
        robotinterface_read_state_blocking();
        robotinterface_unlock_security_stop();
            robotinterface_command_velocity(zero_vector);
            robotinterface_send();
            cout << "Mode: " << robotinterface_get_robot_mode() << endl;
    }
    cout << "Interface opened" << endl;
    cout << "Mode: " << robotinterface_get_robot_mode() << endl;
}

void power_up(){
    cout << "Power up robot" << endl;
    cout << "Mode: " << robotinterface_get_robot_mode() << endl;
    int maxAttemptsToPowerup = 5;
    int retryCounter;
    for (int i = 0; i < maxAttemptsToPowerup; ++i) {
        retryCounter=250;
        robotinterface_read_state_blocking();
        robotinterface_command_empty_command();
        robotinterface_power_on_robot();
        robotinterface_send();
        while (retryCounter > 0) {
            robotinterface_read_state_blocking();
            robotinterface_command_velocity(zero_vector);
            //robotinterface_command_empty_command();
            robotinterface_send();
            if (robotinterface_is_power_on_robot()) {
                break;
            }
            retryCounter--;
        }
    }
    cout << "Powered Up" << endl;
    cout << "Mode: " << robotinterface_get_robot_mode() << endl;
    if (!robotinterface_is_power_on_robot()) {
	    robotinterface_close();
        cout << "Unable to power up robot" << endl;
        exit(EXIT_FAILURE);
    }
}


void initialize(){
	int i = 0;
	cout << "Initializing robot" << endl;
    cout << "Mode: " << robotinterface_get_robot_mode() << endl;
    /// Set zero velocity and acceleration as guard
    for (int j=0; j<6; ++j) {
		pva_packet.velocity[j] = 0.0;
		pva_packet.acceleration[j] = 0.0;
    }
	do {
        ++i;
        robotinterface_read_state_blocking();
        if ((i & 255) == 0) {
            // cout << "Current joint modes:" << endl;
            // print_joint_mode();
            cout << "." ;
        }
        for (int j=0; j<6; ++j) {
            pva_packet.velocity[j] = (robotinterface_get_joint_mode(j) == JOINT_INITIALISATION_MODE) ? -0.15 : 0.0;
        }
        robotinterface_command_velocity(pva_packet.velocity);
        robotinterface_send();
    } while (robotinterface_get_robot_mode() == ROBOT_INITIALIZING_MODE and exit_flag == false);
    cout << " Done!" << endl;
	if (robotinterface_get_robot_mode() != ROBOT_RUNNING_MODE) {
        cout << "Robot in security stop mode, unlocking..." << endl;
        robotinterface_unlock_security_stop();
        robotinterface_unlock_security_stop();
        robotinterface_unlock_security_stop();
        robotinterface_unlock_security_stop();
        robotinterface_unlock_security_stop();
    }
    if (robotinterface_get_robot_mode() != ROBOT_RUNNING_MODE) {
        cout << "Mode: " << robotinterface_get_robot_mode() << endl;
        robotinterface_close();
        cout << "Unable to initialize robot" << endl;
        exit(EXIT_FAILURE);
    }
    // Save original posotion 
    cout << "Sampling initial position" << endl;
    for(i=0;i<10;i++){
        robotinterface_read_state_blocking();
        robotinterface_get_actual_position(pva_packet.position);
        robotinterface_command_velocity(zero_vector);
        //robotinterface_command_position_velocity_acceleration(pva_packet.position, zero_vector, zero_vector);
        robotinterface_send();
    }
    cout << "Initial position: ";
    for (i = 0; i<6; i++) {
        cout << pva_packet.position[i] << " , ";
        /// Ensure that the rest state of the robot is reflected
        pva_packet.velocity[i] = 0.0;
        pva_packet.acceleration[i] = 0.0;
    }
    /// Initialize also the last pva status
    memcpy((void *) &last_pva_packet, (void *) &pva_packet, sizeof(PVAPacket));
    
    cout << endl;
    initialized = true;
}


void settle(){
    /// Warning: It is assumed that the real position has been sampled in pva_packet.position.
    for(int i=0 ; i<125 ; i++){
        if(i%125 == 0) 
            cout << "Waiting for settling, " << i/125 << endl;
        robotinterface_read_state_blocking();
        //robotinterface_get_actual_position(pva_packet.position);
        //robotinterface_command_empty_command();
        if (initialized){
            robotinterface_command_position_velocity_acceleration(pva_packet.position, zero_vector, zero_vector);
        } else{
            robotinterface_command_velocity(zero_vector);
        }
        robotinterface_send();
    }
}


void set_wrench(){
    /// Warning: It is assumed that the real position has been sampled in pva_packet.position.
    robotinterface_read_state_blocking();    
    robotinterface_set_tcp_wrench(tcp_wrench_vector, 1);
    cout << "Is wrench in base? : " << robotinterface_is_tcp_wrench_in_base_coord() << endl;
    if (initialized){
        robotinterface_command_position_velocity_acceleration(pva_packet.position, zero_vector, zero_vector);
    } else {
        robotinterface_command_velocity(zero_vector);
    }
    robotinterface_send();
}
