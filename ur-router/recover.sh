#!/bin/bash
if [[ $EUID -ne 0 ]]; then
  # If not ROOT use sudo; sudo has to be installed
  cd /root
  sudo ./modules_unload eth0
  sudo ./starturcontrol.sh
  cd -
else
  cd /root
  ./modules_unload eth0
  ./starturcontrol.sh
  cd -
fi
