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

#pragma once

#include <robotinterface.h>

/// Types for joint variables
#pragma pack(push,1)
typedef double JointVector[6];
typedef double ToolVector[6];

typedef struct PacketHeader {
    unsigned char protocol_id;
    unsigned long cycle_number;
} PacketHeader;

typedef struct PPacket { //_s_p_packet {
    PacketHeader header;
    JointVector position;
} PPacket ;

typedef struct  PVAPacket { // _s_pva_packet {
    PacketHeader header;
    JointVector position;
    JointVector velocity;
    JointVector acceleration;
} PVAPacket ;

typedef struct ServoPacket {
    PacketHeader header;
    JointVector position;
    JointVector velocity;
} ServoPacket;
#pragma pack(pop)

/// Dynamic and kinematic variables for interaction over network and to robot interface
extern PPacket p_packet;
extern PVAPacket pva_packet;
extern PVAPacket last_pva_packet;
extern ServoPacket servo_packet;
/// Utility vector for setting all values to zero
extern const JointVector zero_vector;
// The wrench that the robot shall exert *on* the tool!
extern ToolVector tcp_wrench_vector;
// The payload on the tcp.
extern double tcp_payload;

/// Logic variables
extern bool exit_flag;

double get_time(void);

class MessageHandler {
protected:
    char string[256];
    int stringpointer;
    struct message_t message;
public:
    MessageHandler();
    void flush();
};
