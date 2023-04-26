#!/bin/bash
sudo chmod 666 /dev/ttyUSB0
gnome-terminal --tab -- bash -ic "cd ~/my_ros/robot_lab; source devel/setup.bash; python3 src/linorobot/scripts/serial_ros.py ; exec bash;"
sleep 1
gnome-terminal --tab -- bash -ic "export TITLE_DEFAULT='keyboard'; cd ~/my_ros/robot_lab; source devel/setup.bash; rosrun robot_model keyboard_teleop_mecanum.py; exec bash;"


