#!/bin/bash
gnome-terminal -e '/bin/bash -c "rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/my_p3at/cmd_vel" ' &