#!/bin/sh
echo Install PrintPupper depend packages...
apt install pigpiod -y
apt install libatlas-base-dev -y
apt install python3-dev -y
apt install python3-pip -y
apt install python3-pigpio -y
apt install python3-numpy -y
pip install serial
pip install UDPComms
pip install ds4drv
pip install transforms3d
echo ...done
