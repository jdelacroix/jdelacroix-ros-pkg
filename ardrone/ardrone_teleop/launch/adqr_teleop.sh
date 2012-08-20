#!/bin/bash
roslaunch ardrone_test.launch ip_addr:=192.168.1.$1 id:=$1
