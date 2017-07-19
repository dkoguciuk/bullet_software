#!/bin/bash

roscore &
sleep 10

roslaunch bullet_kinematics bullet_kinematics.launch &
sleep 5

roslaunch bullet_gait bullet_gait.launch &
sleep 5

roslaunch bullet_teleop bullet_teleop.launch &
sleep 5

rosrun bullet_driver bullet_driver
