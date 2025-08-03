#!/bin/sh
echo "Stop unnecessary services..."
sudo systemctl disable avahi-daemon
sudo systemctl disable hciuart
sudo systemctl disable triggerhappy
sudo systemctl disable apt-daily.timer
sudo systemctl disable apt-daily-upgrade.timer
sudo systemctl mask apt-daily.service
sudo systemctl mask apt-daily-upgrade.service
sleep 1

echo "Disable swap."
sudo swapoff --all
sudo systemctl stop dphys-swapfile
sudo systemctl disable dphys-swapfile
sleep 1

echo "/var/log is on ramdisk."
sudo sed -i '/\/var\/log/ d; $a tmpfs /var/log tmpfs defaults,noatime,nosuid,size=20m,mode=0755 0 0' /etc/fstab
sleep 1

echo "Disable IPv6."
sudo sed -i -e '/^net.ipv6.conf.all.disable_ipv6/d' \
            -e '/^net.ipv6.conf.default.disable_ipv6/d' \
            -e '$a net.ipv6.conf.all.disable_ipv6 = 1\nnet.ipv6.conf.default.disable_ipv6 = 1' \
            /etc/sysctl.conf
sleep 1

echo "Network timeout is short. (5sec)"
sudo nmcli connection modify preconfigured ipv4.dhcp-timeout 5
sudo nmcli connection modify preconfigured connection.wait-device-timeout 5000
sudo nmcli connection show preconfigured | grep ipv4.dhcp-timeout
sudo nmcli connection show preconfigured | grep connection.wait-device-timeout
sleep 1

echo "...done."
echo "After 5sec, reboot..."
sleep 5
echo "Start reboot."
sudo reboot
