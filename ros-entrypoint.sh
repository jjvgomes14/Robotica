#!/bin/bash

source /opt/ros/noetic/setup.bash
catkin_make
source /catkin_ws/devel/setup.bash
exec "$@"