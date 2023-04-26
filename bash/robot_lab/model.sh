#!/bin/bash
cd ~/my_ros/robot_cv
source devel/setup.bash
cd src/model_pkg/scripts/ssd_mobilenet_v2
python3 tracking_deepsort_real.py
