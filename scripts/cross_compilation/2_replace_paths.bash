#!/bin/bash
find ~/pisaBot/arm_rpi/opt/ros/noetic/ -type f -exec sed -i -e 's=/opt/ros/noetic=${CMAKE_CROSS_COMPILE_PREFIX}=g' {} \;