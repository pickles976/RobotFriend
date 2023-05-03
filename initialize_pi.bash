#!/bin/bash

rm -rf build
catkin_make --pkg robot

source ./devel/setup.bash
