#!/bin/bash
# usage xacro2urdf file1.xacro  file2.urdf
echo "Converting file $1 in $2" 
rosrun xacro xacro.py $1 > $2

