#!/bin/bash
# Run ON THE ROBOT (sudo): installs the udev rule for the ESP32 motor driver and applies it to the
# device that is plugged in now. Afterwards the serial port is /dev/gptpet_motors.
set -e
cd "$(dirname "$0")"
sudo install -m 644 99-gptpet-motors.rules /etc/udev/rules.d/99-gptpet-motors.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --subsystem-match=usb
sleep 1
ls -la /dev/gptpet_motors
