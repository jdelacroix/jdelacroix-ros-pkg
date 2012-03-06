#!/bin/bash
rosrun optitrack_driver optitrack __name:=K3_$1 _vrpn_server_ip:=192.168.1.145 _numeric_id:=$1
