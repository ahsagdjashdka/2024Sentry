#!/bin/bash

# Start rm_decision

    # attack_left
    # attack_right
    # retreat_attack_left
    # protect_supply
    # rmuc_01

source install/setup.sh
ros2 launch rm_behavior_tree rm_behavior_tree.launch.py \
    style:=rmuc_01 \
    use_sim_time:=False
