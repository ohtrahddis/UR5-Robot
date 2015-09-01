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

#include <iostream>
#include <cstdio>

using namespace std;

#include "base_utils.hpp"

const JointVector zero_vector = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
PPacket p_packet;
PVAPacket pva_packet;
PVAPacket last_pva_packet;
ServoPacket servo_packet;
// The wrench that the robot shall exert *on* the tool!
ToolVector tcp_wrench_vector = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
// The payload on the tcp.
double tcp_payload = 0.0;
bool exit_flag = false;


MessageHandler::MessageHandler(){
    message.text = string;
}

void MessageHandler::flush(){
    unsigned short i = 0;
    while (robotinterface_get_message_count() > 0) {
        i++;
        stringpointer = 0;
        stringpointer += robotinterface_get_message(&message);
        cout << "Handling message no. " << i << endl;
        cout << message.text << endl;
    } 
}


double get_time (void){
    // Needs -lrt in Makefile
    struct timespec ts;
    if (clock_gettime (CLOCK_MONOTONIC, &ts) < 0) {
        perror ("clock_gettime");
        return -1;
    }
    return (double) ts.tv_sec + ((double) ts.tv_nsec) / 1e9;
}
