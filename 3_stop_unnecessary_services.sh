#!/bin/sh -v
echo Stop unnecessary services.
systemctl stop alsa-restore.service
systemctl disable alsa-restore.service
systemctl stop cron.service
systemctl disable cron.service
systemctl stop dphys-swapfile.service
systemctl disable dphys-swapfile.service
systemctl stop triggerhappy.service
systemctl disable triggerhappy.service
systemctl stop avahi-daemon.service
systemctl disable avahi-daemon.service
systemctl stop console-setup.service
systemctl disable console-setup.service
systemctl stop rsyslog.service
systemctl disable rsyslog.service
systemctl stop ModemManager.service
systemctl disable ModemManager.service
systemctl stop keyboard-setup.service
systemctl disable keyboard-setup.service
systemctl stop rpi-eeprom-update.service
systemctl disable rpi-eeprom-update.service
systemctl stop systemd-random-seed.service
systemctl disable systemd-random-seed.service
systemctl stop systemd-networkd-wait-online.service
systemctl mask systemd-networkd-wait-online.service
echo ...done

