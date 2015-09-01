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

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <netinet/in.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/types.h>
#include <unistd.h>

#include <robotinterface.h>
#include <Configuration.h>
#include <common/microprocessor_commands.h>
#include <common/microprocessor_definitions.h>

#include "startup_utils.hpp"
#include "base_utils.hpp"
#include "interrupt_utils.hpp"


/// The number of tolerated missed response deadlines from external
/// client. Effectively this leaves a time span to start or switch
/// controller in the external client, without losing the connection.
#define TIMEOUT_TOLERANCE 2
#define TIMEOUT_COUNT 0;

using namespace std;

// Communication 
bool is_connected = false;
int rt_connection;
unsigned long timeout_count = TIMEOUT_COUNT;
pthread_mutex_t lock;



void *TcpServerRealTime(void *threadid)
{
    int rt_sock;
    struct sockaddr_in server_addr, client_addr;    
    int sin_size;
    int true_val = 1;  
    // Create socket 
    if ((rt_sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("Socket");
        exit(1);
    }
    // Reuse address
    if (setsockopt(rt_sock, SOL_SOCKET, SO_REUSEADDR, &true_val,sizeof(int)) == -1) {
        perror("Setsockopt");
        exit(1);
    }
    server_addr.sin_family = AF_INET;         
    server_addr.sin_port = htons(5002);     
    server_addr.sin_addr.s_addr = INADDR_ANY; 
    bzero(&(server_addr.sin_zero), 8); 
    // Bind socket
    if (bind(rt_sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr))
        == -1) {
        perror("Unable to bind");
        exit(1);
    }
    // Listen on socket
    if (listen(rt_sock, 5) == -1) {
        perror("Listen");
        exit(1);
    }
    cout << "\nTCPServer Waiting for client on port 5002" << endl;
    fflush(stdout);
    // Thread main loop
    while(true)
        {  
            // Wait for connection
            sin_size = sizeof(struct sockaddr_in);
            cout << "Waiting for connection" << endl;
            rt_connection = accept(rt_sock, (struct sockaddr *)&client_addr, (socklen_t *)&sin_size);
            // Update global variables
            pthread_mutex_lock(&lock);
            is_connected = true;
	    timeout_count = 0;
            pthread_mutex_unlock(&lock);
            // Print status message
            cout << "\n I got a connection from (" <<
                inet_ntoa(client_addr.sin_addr) << " , " 
                 << ntohs(client_addr.sin_port) << ")" << endl;
            fflush(stdout);
            // Check connection status variable
            bool conn;
            pthread_mutex_lock(&lock);
            conn = is_connected;
            pthread_mutex_unlock(&lock);
            // Loop until conection is lost
            while (conn == true)
            {
                pthread_mutex_lock(&lock);
                conn = is_connected;
                pthread_mutex_unlock(&lock);
            }
        }       
    cout << "Close Thread" << endl;
    // Close socket
    close(rt_sock);
    pthread_exit(NULL);
} 

void *TcpServerDigitalOut(void *threadid)
{
    int io_sock;
    struct sockaddr_in server_addr, client_addr;    
    int connection_do;
    int read_size;
    int sin_size;
    char buffer[1024];
    int true_val = 1;  
    // Create socket
    if ((io_sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) 
    {
        perror("Socket");
        exit(1);
    }
    // Reuse socket address
    if (setsockopt(io_sock, SOL_SOCKET, SO_REUSEADDR, &true_val, sizeof(int)) == -1) 
    {
        perror("Setsockopt");
        exit(1);
    }
    server_addr.sin_family = AF_INET;         
    server_addr.sin_port = htons(5003);
    server_addr.sin_addr.s_addr = INADDR_ANY; 
    bzero(&(server_addr.sin_zero),8); 
    // Bind Socket
    if (bind(io_sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1) 
    {
        perror("Unable to bind");
        exit(1);
    }
    // Listen on socket 
    if (listen(io_sock, 5) == -1) {
        perror("Listen");
        exit(1);
    }
    cout << "\nTCPServer Waiting for client on port 5003 for Digital Output set" << endl;
    fflush(stdout);
    while(true)
        {  
            sin_size = sizeof(struct sockaddr_in);
            cout << "Waiting for control connection" << endl;
            // Wait for connection
            connection_do = accept(io_sock, (struct sockaddr *)&client_addr,(socklen_t *)&sin_size);
            cout << "\n I got a control connection from (" <<
                inet_ntoa(client_addr.sin_addr)<< " , " 
                 <<ntohs(client_addr.sin_port) << ")" << endl;
            fflush(stdout);
            // Read commands from connection
            while(true)
            {
                read_size = read (connection_do, buffer, 1024);
                if (read_size < 1)
                {
                    break;
                }
                else
                {
                    if (read_size == 2)
                    {
                        int port = 0;
                        port += buffer[0];
                        int value = 0;
                        value += buffer[1];
                        cout << "Setting digital out" << endl;
                        robotinterface_command_digital_out_port(port, value);
                    }
                }
            }
            cout << "TCP Digital Out disconnected" << endl;
        }       
    cout << "Close TCP Digital Out Thread" << endl;
    // Close Socket
    close(io_sock);
    pthread_exit(NULL);
} 

int main(int argc, char *argv[]) {
    unsigned short j;
    unsigned long cycle_number=0;
    double control_time, init_time;
    bool timeout_flag = false;
    pthread_t threads[2];
    long t = 0;
    int rc;
    const unsigned long timeout_tolerance = TIMEOUT_TOLERANCE; //ULONG_MAX;
    MessageHandler msg_handler;

    rc = pthread_create(&threads[0], NULL, TcpServerRealTime, (void *)t);
    rc = pthread_create(&threads[1], NULL, TcpServerDigitalOut, (void *)t);
    pthread_mutex_init(&lock,0);
    setup_sigint(); // Enable catching of SIGINT
    setup_sigpipe(); // Enable catching of SIGPIPE
    cout << "=== Starting routing program ===" << endl;
    cout << "- Loading robot configuration" << endl;
    configuration_load();
    cout << "Setting tool payload: " << tcp_payload << endl;
    robotinterface_set_tcp_payload(tcp_payload);
    //configuration_print();
    //extern struct Configuration conf;
    cout << "Gravity (Tool mass) : " << conf.gravity[0] << ", " << conf.gravity[1] << ", " << conf.gravity[2] << " (" << conf.mass[5] << ")"<< endl;
    
    open_interface();
    power_up();
    set_wrench();
    initialize();
    settle();

    for (j = 0; j < 6; j++) 
    {
        last_pva_packet.position[j] = pva_packet.position[j];
    }

    cout << "=== Starting main loop ===" << endl;
    init_time = get_time();
    // Main Loop
    char buffer[1024];
    int read_size;

    for (cycle_number = 0; exit_flag == false; cycle_number++) {
        robotinterface_read_state_blocking();
        control_time = get_time() - init_time;
        // Read robot data and write to buffer
        robotinterface_get_actual(servo_packet.position, servo_packet.velocity);
        pthread_mutex_lock (&lock);
        
        if (is_connected) {
            servo_packet.header.protocol_id = 0x00;
            servo_packet.header.cycle_number = cycle_number;
            int ret = send(rt_connection, (void *) &servo_packet, sizeof(ServoPacket), 0); 
            if (ret <0) {
                cout << "Error in write: " << ret << endl << flush;
                is_connected = false;
                close(rt_connection);
            }
        }
	timeout_flag = false;

        if (is_connected) {
            fd_set masterfds, readfds;
            struct timeval timeout;
            FD_ZERO(&masterfds);
            FD_SET(rt_connection, &masterfds);
            memcpy(&readfds, &masterfds, sizeof(fd_set));
            timeout.tv_sec = 0;
            timeout.tv_usec = 6000;
            if (select(rt_connection + 1, &readfds, NULL, NULL, &timeout) < 0) {
                cout << "Select error. Exitting!" << endl;
                exit(1);
            }
            if (FD_ISSET(rt_connection, &readfds)) {
                read_size = read (rt_connection, buffer, 1024);
            } else {            
                // The socket recv timed out
                read_size = -2;
                // if(cycle_number%125==0){
                    cout << "Socket recv timeout." << endl;
		    //}
		timeout_flag = true;
		timeout_count++;
		
            }
            if (!timeout_flag){
		if(read_size == sizeof(PPacket) || read_size == sizeof(PVAPacket) ) {
		    // Read packet data into the pva_packet. Later
		    // inter-extra-polation is done, if only a PPacket
		    // was given.
		    memcpy((void *) &pva_packet, (void *) buffer, read_size);
		    if (pva_packet.header.cycle_number != cycle_number) {
			cout << "Error!!!! Sequence error" << endl;
			//is_connected = false;
			//close(rt_connection);
		    }
		    timeout_count = 0;
		} 
                else 
                { /// Got wrong read_size
		    cout << "Wrong data size: "<< read_size << endl;
		    is_connected = false;
		    close(rt_connection);
		}
	    }
	}
        if (is_connected)
        {
            if(!timeout_flag)
            {
		/// Inter-extra-polate velocities and accelerations,
		/// if only a position packet was received
		if (read_size == sizeof(PPacket))
                {
		    for (j = 0; j<6; j++) {
			pva_packet.velocity[j] = (pva_packet.position[j] - last_pva_packet.position[j]) / 0.008;
			pva_packet.acceleration[j] = (pva_packet.velocity[j] - last_pva_packet.velocity[j]) / 0.008;
		    }
		}
		robotinterface_command_position_velocity_acceleration(pva_packet.position,
								      pva_packet.velocity,
								      pva_packet.acceleration);
		// Save sent position to use in in case of error
		for (j = 0; j<6; j++) {
		    last_pva_packet.position[j] = pva_packet.position[j];
		    last_pva_packet.velocity[j] = pva_packet.velocity[j];
		}
	    } else { /// Timeout
		robotinterface_command_velocity(zero_vector);
		if(timeout_count > timeout_tolerance){
		    /// Tollerance for timeouts exceeded, disconnect
		    is_connected = false;
		    close(rt_connection);
		}
	    }
        } 
        else 
        {
            if (cycle_number%125 == 0 )
                cout << "No connection" << endl;
            for (j = 0; j < 6; j++) 
            {
                last_pva_packet.position[j] = pva_packet.position[j];
                last_pva_packet.velocity[j] = 0;
            }
            robotinterface_command_velocity(zero_vector);
        }
        pthread_mutex_unlock (&lock);
        // Commit changes
        robotinterface_send();
        // Flush messages to stdout
        msg_handler.flush();
    }
    cout << "- Speeding down" << endl;
    for (j = 0; j < 10; j++) 
    {
        robotinterface_read_state_blocking();
        robotinterface_command_velocity(zero_vector);
        robotinterface_send();
    }
    cout << "- Closing" << endl;
    robotinterface_close();
    //kill(getpid(), SIGTERM);
    pthread_cancel(threads[0]);
    pthread_cancel(threads[1]);
    cout << "Cancel TCP Thread." << endl;
    pthread_exit(NULL);
    cout << "Done." << endl;
    return 0;
}

