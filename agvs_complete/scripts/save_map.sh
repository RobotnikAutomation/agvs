#!/bin/sh
# parameter $1: name of the map

rosrun map_server map_saver -f $1
