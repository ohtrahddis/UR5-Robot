#!/bin/bash
if [[ $EUID -ne 0 ]]; then
  # If not ROOT use sudo; sudo has to be installed
  sudo /root/stopurcontrol.sh
  cd /root
  sudo ./modules_load eth0
  cd $HOME/ur-router
  sudo ./router_tcp
  cd /root
  sudo ./modules_unload eth0
  sudo ./starturcontrol.sh
  cd
else
  /root/stopurcontrol.sh
  cd /root
  ./modules_load eth0
  cd $HOME/ur-router
  ./router_tcp
  cd /root
  ./modules_unload eth0
  ./starturcontrol.sh
  cd
fi
