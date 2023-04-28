#!/bin/bash

# TODO launch both rover launch files from here

source install/setup.bash
ros2 launch home_launch.xml &
ros2 launch rover_launch.xml &