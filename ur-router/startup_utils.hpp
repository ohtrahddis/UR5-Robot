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

void open_interface();
void power_up();
void initialize();
void settle();
void set_wrench();
