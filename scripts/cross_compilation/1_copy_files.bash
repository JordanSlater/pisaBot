#!/bin/bash
echo -n "deleting ... "
rm -r ~/pisaBot/arm_rpi/
echo "done"

mkdir -p ~/pisaBot/arm_rpi/opt/ros/

echo "beginning scp (secure copy):"
scp -r ubuntu@192.168.2.47:/opt/ros/noetic/* ~/pisaBot/arm_rpi/opt/ros/noetic/
echo "done"