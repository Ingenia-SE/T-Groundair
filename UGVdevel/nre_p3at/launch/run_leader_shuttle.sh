#!/bin/bash
# e.g., export BASE_IP=192.168.11.120
if [ "$#" -lt 1 ]; then
    echo "ERROR: Must supply the IP address of the ROS master as an argument!"
    echo "EXAMPLE:"
    echo "   rosrun nre_p3at 192.168.11.120"
    exit -1
fi
export ROS_MASTER_URI=http://$1:11311
export ROBOT_IP=$(ifconfig wlan0 | grep 'inet addr' | cut -d: -f2 | awk '{print $1}')
export ROS_IP=${ROBOT_IP}
export ROS_HOSTNAME=${ROBOT_IP}
echo "----------------------------------------"
env | grep ROS
echo "----------------------------------------"

roslaunch nre_p3at onep3at.shuttle.launch namespace:=leader
