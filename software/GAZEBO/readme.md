UAV + ENVIRONMENT + SERVER

Simulation of robot system with a bebop2 drone and a server, with a real environment.

Installation process:

1) Copy pkgs into your ROS_WS
2) Install dependencies:

>> rosdep install --from-paths src -i

In case you have another ROS_DISTRO, replace the word melodic with your DISTRO:

>> rosdep install --from-paths src -i --rosdistro melodic --os=ubuntu:bionic

3) Build the entire workspace

>> catkin build

NOTES: Three pkgs will be ignored.

Follow next steps to simulate the entire system:

>> roslaunch bebop_simulator simulation.launch


