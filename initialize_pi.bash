#!/bin/bash

rm -rf build
catkin_make -only-pkg-with-deps robot std_msgs

source ./devel/setup.bash

cd src/robot/src/imu-rs
cargo run
cd ../../../..