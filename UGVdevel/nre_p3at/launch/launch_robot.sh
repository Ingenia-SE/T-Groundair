#/bin/bash
export BASE_IP=192.168.11.120
export ROS_MASTER_URI=http://${BASE_IP}:11311
export ROBOT_IP=$(ifconfig wlan0 | grep 'inet addr' | cut -d: -f2 | awk '{print $1}')
export ROS_IP=${ROBOT_IP}
export ROS_HOSTNAME=${ROBOT_IP}
rosrun rosaria RosAria _port:=/dev/ttyS0 __name:=my_p3at
rosrun nmea_navsat_driver nmea_serial_driver _port:=/dev/ttyS1 _baud:=4800
#rosrun um7 um7_driver _port:=/dev/ttyUSB0
roslaunch base2.launch

export BASE_IP=192.168.11.120
export ROS_MASTER_URI=http://${BASE_IP}:11311
export ROBOT_IP=$(ifconfig wlan0 | grep 'inet addr' | cut -d: -f2 | awk '{print $1}')
export ROS_IP=${ROBOT_IP}
export ROS_HOSTNAME=${ROBOT_IP}
roslaunch nre_p3at shuttle.launch


export BASE_IP=192.168.11.120
export ROS_MASTER_URI=http://${BASE_IP}:11311
export ROBOT_IP=$(ifconfig wlan0 | grep 'inet addr' | cut -d: -f2 | awk '{print $1}')
export ROS_IP=${ROBOT_IP}
export ROS_HOSTNAME=${ROBOT_IP}
rosbag record -a -O waypoint.bag
