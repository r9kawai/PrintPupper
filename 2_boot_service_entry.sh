#!/bin/sh
echo Entry robot.service and joystick.service ...
sudo ln -s /home/pi/PrintPupper/src/PupperCommand/joystick.service /etc/systemd/system/joystick.service
sudo ln -s /home/pi/PrintPupper/src/robot.service /etc/systemd/system/robot.service
systemctl daemon-reload
systemctl enable joystick.service
systemctl enable robot.service
systemctl start joystick
systemctl start robot
systemctl status joystick
systemctl status robot
echo ...done
