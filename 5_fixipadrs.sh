#!/bin/sh
echo Set static ip 192.168.11.111

sudo sed -i -e '$ainterface eth0' /etc/dhcpcd.conf
sudo sed -i -e '$astatic ip_address=192.168.11.111' /etc/dhcpcd.conf
sudo sed -i -e '$astatic routers=192.168.1.1' /etc/dhcpcd.conf
sudo sed -i -e '$astatic domain_name_servers=192.168.1.1' /etc/dhcpcd.conf

echo please reboot
