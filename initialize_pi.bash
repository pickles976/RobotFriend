#!/bin/bash

rm -rf build
catkin_make -only-pkg-with-deps robot

source ./devel/setup.bash

cd src/robot/src/imu-rs
cargo run
cd ../../../..