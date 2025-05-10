#!/bin/bash

ros2 service call /motor_control_service gptpet_common/srv/MotorControl "{motor_speeds: {velocity_left_1: -0.7, velocity_left_2: -0.7, velocity_right_1: -0.7, velocity_right_2: -0.7}}"