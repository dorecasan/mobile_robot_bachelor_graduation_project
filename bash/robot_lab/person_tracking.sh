#!/bin/bash
cd ~/my_ros/robot_cv
source devel/setup.bash
cd src/model_pkg/scripts/ssd_mobilenet_v2
python3 tracking_deepsort_real.py & gnome-terminal --tab -- bash -ic "export TITLE_DEFAULT='camera'; cd ~/my_ros/bash/robot_lab; ./camera.sh; exec bash;"  & gnome-terminal --tab -- bash -ic "export TITLE_DEFAULT='control'; cd ~/my_ros/bash/robot_lab; ./control.sh; exec bash;" 
