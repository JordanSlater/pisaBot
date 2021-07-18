#!/bin/bash
find ~/pisaBot/arm_rpi/opt/ros/noetic/ -type f -exec sed -i -e 's=/opt/ros/noetic=${CMAKE_CROSS_COMPILE_PREFIX}=g' {} \;

sed 's\;thread;\;${CMAKE_CROSS_COMPILE_PREFIX}/usr/lib/aarch64-linux-gnu/libpthread.so.0;\g' ~/pisaBot/arm_rpi/opt/ros/noetic/share/roscpp/cmake/roscppConfig.cmake > test.txt

