## INGENIA-SE ROS package
[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

Here you can find ROS package for [INGENIA-SE project].
Follow next steps to install and run the simulation. Package include:
  - Enviroment
  - UAV controller
  - UGV controller
  - Server

# 1. Installation procedure
1) Git clone INGENIA-SE repo and move software file into your ROS_WS
2) Install dependencies:
    ```sh
    $ rosdep install --from-paths src -i
    ```
    In case you have another ROS_DISTRO, replace the word melodic with your DISTRO:
    ```sh
    $ rosdep install --from-paths src -i --rosdistro melodic --os=ubuntu:bionic
    ```
3) Build theworkspace
    ```sh
    $ catkin build
    ```
NOTES: Three pkgs will be ignored.
# 2. Running simulation
Despite it is a complex system based on multiple packages, simulation can be runned just with this command.
```sh
    $ roslaunch bebop_simulator simulation.launch
```
