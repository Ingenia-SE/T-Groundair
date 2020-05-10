Quickstart guide:

>Follow tutorial to create catkin_ws http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Install AMR-ROS-CONFIG

* cd ~/catkin_ws/src
* git clone https://github.com/MobileRobots/amr-ros-config.git
* cd ~/catkin_ws
* catkin_make
* source ~/catkin_ws/devel/setup.bash

Install nre_p3at

* cd ~/catkin_ws/src
* git clone https://github.com/bsb808/nre_p3at.git
* Re-setup your ros environment so that the system knows about our new package
* source ~/catkin_ws/devel/setup.bash
* If everything is working, this should put you in the package directory
* roscd nre_p3at

Install ugv_controller
* Download this branch and place ugv_controller in /catkin_ws/src
* cd ~/catkin_ws
* catkin_make
* source ~/catkin_ws/devel/setup.bash

Run programs
* Run roslaunch /nre_p3at/launch/p3at.gazebo.launch
* Run rosrun ugv_controller position_control.py
