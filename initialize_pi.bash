#!/bin/bash

rm -rf build
catkin_make --pkg robot

pigpiod

source ./devel/setup.bash
