#!/bin/bash

# Start navigation
source install/setup.sh

ros2 launch rm_bringup bringup_real.launch.py \
    world:=RMUC \
    mode:=nav \
    lio:=fastlio \
    localization:=slam_toolbox \
    lio_rviz:=False \
    nav_rviz:=True
