export BASE_IP=$(ifconfig wlan0 | grep 'inet addr' | cut -d: -f2 | awk '{print $1}')
if [ "${#BASE_IP}" -lt 14 ]; then
    echo "ERROR! You must be connected to wifi."
    echo "Check \"ifconfig wlan0\" "
    exit -1
fi
echo "Setting environmental variable for BASE_IP="$BASE_IP
export ROS_MASTER_URI=http://${BASE_IP}:11311
export ROS_IP=${BASE_IP}
export ROS_HOSTNAME=${BASE_IP}
echo "----------------------------------------"
env | grep ROS
echo "----------------------------------------"
roslaunch nre_p3at rviz_leaderfollower.launch
