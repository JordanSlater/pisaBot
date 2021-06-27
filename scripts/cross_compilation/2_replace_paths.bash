#!/bin/bash
sed -i 's=/opt/ros/noetic=${CMAKE_CROSS_COMPILE_PREFIX}=g' ~/pisaBot/arm_rpi/opt/ros/noetic/*