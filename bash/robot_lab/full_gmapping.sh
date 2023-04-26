#!/bin/bash

gnome-terminal -- bash -ic "cd ~/my_ros/bash/robot_lab; ./control.sh; exec bash;"
sleep 1
#gnome-terminal --tab -- bash -ic "export TITLE_DEFAULT='title2'; cd ~/my_ros/bash/robot_lab; ./rtabmap.sh; exec bash;"
gnome-terminal --tab -- bash -ic "export TITLE_DEFAULT='title2'; cd ~/my_ros/bash/robot_lab; ./gmapping.sh; exec bash;"
