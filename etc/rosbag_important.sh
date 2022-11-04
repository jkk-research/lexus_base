cd /mnt/bag/
mkdir -p $(date -I)
cd $(date -I)
TEXT1="$1"
TIME1="$(date +"%Y-%m-%d_%H-%M")"
FILE1="$TEXT1$TIME1.bag"
FILE2="x_rosparam_dump_$TEXT1$TIME1.txt"
PWD1="$(pwd)"
echo ""
echo "Writing to file: $PWD1/$FILE1"
echo ""
echo "Check it after measurement with foxglove or:"
echo "rosbag info $PWD1/$FILE1"
echo ""
#rosparam dump $FILE2
rosbag record -O $FILE1 -b 4096 /tf /gps/duro/current_pose /gps/duro/imu /gps/duro/status_flag /gps/duro/status_string /os_center/points /os_right/points /zed_node/left/camera_info /zed_node/left/image_rect_color /pacmod/parsed_tx/accel_rpt /pacmod/parsed_tx/brake_rpt /pacmod/parsed_tx/steer_rpt
#rosbag record -O ns_$FILE1 -b 4096 /tf /lexus1/gps/duro/current_pose /lexus1/gps/duro/imu /lexus1/gps/duro/status_flag /lexus1/gps/duro/status_string /lexus1/os_center/points /lexus1/os_right/points /lexus1/zed_node/left/camera_info /lexus1/zed_node/left/image_rect_color /lexus1/pacmod/parsed_tx/accel_rpt /lexus1/pacmod/parsed_tx/brake_rpt /lexus1/pacmod/parsed_tx/steer_rpt
