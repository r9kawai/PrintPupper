#!/bin/sh
echo Set var, tmp, log to ramdisk.

sudo sed -i -e '$atmpfs /tmp tmpfs defaults,size=16m,noatime,mode=1777 0 0' /etc/fstab
sudo sed -i -e '$atmpfs /var/tmp tmpfs defaults,size=16m,noatime,mode=1777 0 0' /etc/fstab
sudo sed -i -e '$atmpfs /var/log tmpfs defaults,size=16m,noatime,mode=0755 0 0' /etc/fstab

sudo rm -rf /tmp
sudo rm -rf /var/tmp
sudo rm -rf /var/log

echo ...done, Please reboot
