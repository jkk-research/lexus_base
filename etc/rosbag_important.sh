cd ~/bag/
mkdir -p $(date -I)
cd $(date -I)
TEXT1="$1"
TIME1="$(date +"%Y-%m-%d_%H-%M")"
FILE1="$TEXT1$TIME1.bag"
FILE2="x_rosparam_dump_$TEXT1$TIME1.txt"
PWD1="$(pwd)"
echo "Writing to file: $PWD1/$FILE1"
#rosparam dump $FILE2
rosbag record -O $FILE1 -b 4096 /tf /gps/duro/current_pose /gps/duro/imu /gps/duro/status_flag /gps/duro/status_string /os_center/points /os_right/points /zed_node/left/camera_info /zed_node/left/image_rect_color /pacmod/parsed_tx/accel_rpt /pacmod/parsed_tx/brake_rpt /pacmod/parsed_tx/steer_rpt
