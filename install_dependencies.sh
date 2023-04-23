#!/bin/bash

# run 'apt-get update' before call this script
apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-franka-description \
    ros-$ROS_DISTRO-franka-gazebo \
    ros-$ROS_DISTRO-franka-msgs \
    python3-yaml \
    python3-rospkg