#!/usr/bin/env bash

source /home/orangepi/ros2_humble/install/setup.bash
source /home/orangepi/CityScout/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/orangepi/CityScout/deploy/cyclonedds.xml
