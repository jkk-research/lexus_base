#!/bin/bash
source /opt/ros/melodic/setup.bash
source ~/standard_ws/devel/setup.bash
#source ~/catkin_ws/devel/setup.bash

export ROS_IP=192.168.1.5
export ROS_MASTER_URI=http://192.168.1.5:11311

export DISPLAY=:0
echo "[INFO] Starting roscore"
screen -d -m -S roscore bash -c 'roscore'
xrandr --output HDMI-0 --mode 1280x720
sleep 4
echo "[INFO] Starting zed_default"
screen -d -m -S driver_zed2_def bash -c 'roslaunch lexus_base zed_default.launch'
echo "[INFO] Starting gps_duro"
screen -d -m -S driver_duro_gps bash -c 'roslaunch lexus_base gps_duro.launch'
echo "[INFO] Starting teleop_on_vehicle_CAN_TF"
screen -d -m -S teleop_on_vehicle_CAN_TF bash -c 'roslaunch lexus_base teleop_on_vehicle.launch'
echo "[INFO] Starting rqt_image_view"
screen -d -m -S view_zed_cam bash -c 'rosrun rqt_image_view rqt_image_view /zed_node/left/image_rect_color --on-top '
#screen -d -m -S view_zed_cam bash -c 'rosrun rqt_image_view rqt_image_view /zed_node/left/image_rect_color/compressed --on-top '
echo "[INFO] Starting gui_teleop1"
screen -d -m -S teleop_gui_s bash -c 'rosrun lexus_base gui_teleop1.py'
echo "[INFO] Starting rviz"
screen -d -m -S teleopg_rviz bash -c 'roslaunch lexus_base rviz00.launch'
echo "[INFO] Starting lanelet_marker_zala_uni"
screen -d -m -S lanelet_marker_zala_uni bash -c 'roslaunch lexus_base lanelet_marker_zala_uni.launch'
echo "[INFO] OK"
sleep 4
#wmctrl -r :ACTIVE: -e 5,0,0,1280,500