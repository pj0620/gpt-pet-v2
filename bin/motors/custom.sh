#!/bin/bash

ros2 service call /motor_control common_interfaces/srv/MotorControl "{motor_speeds: {velocity_left_1: 0.0, velocity_left_2: 0.0, velocity_right_1: 0.0, velocity_right_2: 0.7}}"