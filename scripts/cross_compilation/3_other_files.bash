#!/bin/bash
mkdir -p ~/pisaBot/arm_rpi/usr/lib/aarch64-linux-gnu/
# mkdir -p ~/pisaBot/arm_rpi/include/

echo "Copy /usr/lib/aarch64-linux-gnu/librt*"
scp ubuntu@192.168.2.47:/usr/lib/aarch64-linux-gnu/librt* ~/pisaBot/arm_rpi/usr/lib/aarch64-linux-gnu/

echo "Copy /usr/lib/aarch64-linux-gnu/librt*"
scp ubuntu@192.168.2.47:/usr/lib/aarch64-linux-gnu/libpthread.so.0 ~/pisaBot/arm_rpi/usr/lib/aarch64-linux-gnu/


# /usr/lib/libMPU6050.so 
# /usr/include/MPU6050.h
# /usr/include/i2c/smbus.h
# /usr/lib/aarch64-linux-gnu/libpthread.so

# In /home/jordan/pisaBot/arm_rpi/opt/ros/noetic/roscpp/cmake/roscppConfig.cmake
#   replace ;thread; with ;${CMAKE_CROSS_COMPILE_PREFIX}/usr/lib/aarch64-linux-gnu/libpthread.so.0;
