#!/bin/bash
source /opt/ros/melodic/setup.bash
source ~/standard_ws/devel/setup.bash
#source ~/catkin_ws/devel/setup.bash

export ROS_IP=192.168.1.5
export ROS_MASTER_URI=http://192.168.1.5:11311

export DISPLAY=:0
echo "[INFO] Starting roscore"
screen -d -m -S roscore bash -c 'roscore'
sleep 4
echo "[INFO] Starting zed_default"
screen -d -m -S driver_zed2_def bash -c 'roslaunch lexus_base zed_default.launch'
echo "[INFO] Starting gps_duro"
screen -d -m -S driver_duro_gps bash -c 'roslaunch lexus_base gps_duro.launch'
echo "[INFO] Starting tf_static"
screen -d -m -S tf_static bash -c 'roslaunch lexus_base tf_static.launch'
echo "[INFO] Starting 3D"
screen -d -m -S 3d bash -c 'roslaunch lexus_base 3d.launch'
echo "[INFO] Starting rviz"
screen -d -m -S rviz bash -c 'roslaunch lexus_base rviz01static.launch'
echo "[INFO] OK"
sleep 4
#wmctrl -r :ACTIVE: -e 5,0,0,1280,500