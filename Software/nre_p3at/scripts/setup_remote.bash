
# Change the following variable to the IP address of your laptop.
MYIP=192.168.11.120
# Change this to the IP address of the robot
ROBOTIP=192.168.11.60

export ROS_MASTER_URI=http://${ROBOTIP}:11311
export ROS_IP=${MYIP}
export ROS_HOSTNAME=${MYIP}