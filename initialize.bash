#!/bin/bash

rm -rf build
catkin_make
pigpiod

source ./devel/setup.bash
