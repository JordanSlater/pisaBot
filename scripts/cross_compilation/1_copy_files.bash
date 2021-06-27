#!/bin/bash
echo -n "deleting ... "
rm -r ~/pisaBot/arm_rpi/
echo "done"

mkdir -p ~/pisaBot/arm_rpi/opt/ros/

echo -n "copying ... "
cp -r /opt/ros/noetic/ ~/pisaBot/arm_rpi/opt/ros/noetic/
echo "done"