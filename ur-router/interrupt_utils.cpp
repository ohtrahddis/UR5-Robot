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

#include <csignal>
#include <cstdio>
#include <cstdlib>

using namespace std;

#include "base_utils.hpp"

static void sigint_handler (int signal, siginfo_t * info, void *data) { 
    printf ("SIGINT\n");
    fflush(stdout);
    exit_flag = true;
    //signal(SIGINT, SIG_DFL);
}

static void sigpipe_handler (int signal, siginfo_t * info, void *data) { 
    printf ("SIGPIPE\n");
    fflush(stdout);
    //signal(SIGINT, SIG_DFL);
}


void  setup_sigint(){
    struct sigaction act;
    act.sa_sigaction = sigint_handler;
    act.sa_flags = SA_RESTART | SA_SIGINFO;
    sigemptyset (&act.sa_mask);
    if (sigaction (SIGINT, &act, NULL) < 0){
        perror ("sigaction");
        exit(0);
    }
}

void  setup_sigpipe(){
    struct sigaction act2;
    act2.sa_sigaction = sigpipe_handler;
    act2.sa_flags = SA_RESTART | SA_SIGINFO;
    sigemptyset (&act2.sa_mask);
    if (sigaction (SIGPIPE, &act2, NULL) < 0) {
        perror ("sigaction");
        exit(0);
    }
}
