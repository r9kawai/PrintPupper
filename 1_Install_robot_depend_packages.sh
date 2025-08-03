#!/bin/sh
echo "Install PrintPupper depend packages..."
sudo apt install pigpiod -y
sudo apt install libatlas-base-dev -y
sudo apt install python3-dev -y
sudo apt install python3-pip -y
sudo apt install python3-pigpio -y
sudo apt install python3-numpy -y
sudo pip install serial
sudo pip install UDPComms
sudo pip install ds4drv
sudo pip install transforms3d
echo "...done"
