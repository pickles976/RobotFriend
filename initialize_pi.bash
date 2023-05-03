#!/bin/bash

rm -rf build
catkin_make --pkg ros_numpy robot

source ./devel/setup.bash
