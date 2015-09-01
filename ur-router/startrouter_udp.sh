#!/bin/sh
cd /root/ur-router
../stopurcontrol.sh
cd ..
./modules_load eth0
cd -
./router_udp
cd ..
./modules_unload eth0
./starturcontrol.sh
cd -
