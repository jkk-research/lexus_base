cd /mnt/bag/
mkdir -p $(date -I)
cd $(date -I)
TEXT1="$1_ns_"
TIME1="$(date +"%Y-%m-%d_%H-%M")"
FILE1="$TEXT1$TIME1.bag"
FILE2="x_rosparam_dump_$TEXT1$TIME1.txt"
PWD1="$(pwd)"
echo ""
echo "Writing to file: $PWD1/ns_$FILE1"
echo ""
echo "Check it after measurement with foxglove or:"
echo "rosbag info $PWD1/$FILE1"
echo ""
#rosparam dump $FILE2
#rosbag record -O $FILE1 -b 4096 /tf /gps/duro/current_pose /gps/duro/imu /gps/duro/status_flag /gps/duro/status_string /os_center/points /os_right/points /zed_node/left/camera_info /zed_node/left/image_rect_color /pacmod/parsed_tx/accel_rpt /pacmod/parsed_tx/brake_rpt /pacmod/parsed_tx/steer_rpt
rosbag record -O $FILE1 -b 4096 /diagnostics /lexus3/gps/duro/current_pose /lexus3/gps/duro/current_pose_fake_orientation /lexus3/gps/duro/fix /lexus3/gps/duro/imu /lexus3/gps/duro/mag /lexus3/gps/duro/odom /lexus3/gps/duro/rollpitchyaw /lexus3/gps/duro/rollpitchyaw_fake /lexus3/gps/duro/status_flag /lexus3/gps/duro/status_string /lexus3/gps/duro/time_diff /lexus3/gps/duro/time_gps_str /lexus3/gps/duro/time_ref /lexus3/lexus_f/lexus_marker /lexus3/os_center/imu /lexus3/os_center/points /lexus3/os_right/imu /lexus3/os_right/points /lexus3/pacmod/as_rx/accel_cmd /lexus3/pacmod/as_rx/brake_cmd /lexus3/pacmod/as_rx/headlight_cmd /lexus3/pacmod/as_rx/horn_cmd /lexus3/pacmod/as_rx/rear_pass_door_cmd /lexus3/pacmod/as_rx/shift_cmd /lexus3/pacmod/as_rx/steer_cmd /lexus3/pacmod/as_rx/turn_cmd /lexus3/pacmod/as_tx/all_system_statuses /lexus3/pacmod/as_tx/enabled /lexus3/pacmod/as_tx/vehicle_speed /lexus3/pacmod/can_rx /lexus3/pacmod/can_tx /lexus3/pacmod/parsed_tx/accel_aux_rpt /lexus3/pacmod/parsed_tx/accel_rpt /lexus3/pacmod/parsed_tx/brake_aux_rpt /lexus3/pacmod/parsed_tx/brake_rpt /lexus3/pacmod/parsed_tx/component_rpt /lexus3/pacmod/parsed_tx/date_time_rpt /lexus3/pacmod/parsed_tx/global_rpt /lexus3/pacmod/parsed_tx/headlight_aux_rpt /lexus3/pacmod/parsed_tx/headlight_rpt /lexus3/pacmod/parsed_tx/horn_rpt /lexus3/pacmod/parsed_tx/lat_lon_heading_rpt /lexus3/pacmod/parsed_tx/parking_brake_status_rpt /lexus3/pacmod/parsed_tx/rear_pass_door_rpt /lexus3/pacmod/parsed_tx/shift_aux_rpt /lexus3/pacmod/parsed_tx/shift_rpt /lexus3/pacmod/parsed_tx/steer_aux_rpt /lexus3/pacmod/parsed_tx/steer_rpt /lexus3/pacmod/parsed_tx/turn_aux_rpt /lexus3/pacmod/parsed_tx/turn_rpt /lexus3/pacmod/parsed_tx/vehicle_speed_rpt /lexus3/pacmod/parsed_tx/vin_rpt /lexus3/pacmod/parsed_tx/wheel_speed_rpt /lexus3/pacmod/parsed_tx/yaw_rate_rpt /lexus3/zed_node/left/image_rect_color /rosout /rosout_agg /tf /tf_static 

#/diagnostics
#/lexus3/gps/duro/current_pose
#/lexus3/gps/duro/current_pose_fake_orientation
#/lexus3/gps/duro/fix
#/lexus3/gps/duro/imu
#/lexus3/gps/duro/mag
#/lexus3/gps/duro/odom
#/lexus3/gps/duro/rollpitchyaw
#/lexus3/gps/duro/rollpitchyaw_fake
#/lexus3/gps/duro/status_flag
#/lexus3/gps/duro/status_string
#/lexus3/gps/duro/time_diff
#/lexus3/gps/duro/time_gps_str
#/lexus3/gps/duro/time_ref
#/lexus3/lexus_f/lexus_marker
#/lexus3/os_center/imu
#/lexus3/os_center/imu_packets
#/lexus3/os_center/lidar_packets
#/lexus3/os_center/nearir_image
#/lexus3/os_center/os_nodelet_mgr/bond
#/lexus3/os_center/points
#/lexus3/os_center/range_image
#/lexus3/os_center/reflec_image
#/lexus3/os_center/signal_image
#/lexus3/os_right/imu
#/lexus3/os_right/imu_packets
#/lexus3/os_right/lidar_packets
#/lexus3/os_right/nearir_image
#/lexus3/os_right/os_nodelet_mgr/bond
#/lexus3/os_right/points
#/lexus3/os_right/range_image
#/lexus3/os_right/reflec_image
#/lexus3/os_right/signal_image
#/lexus3/pacmod/as_rx/accel_cmd
#/lexus3/pacmod/as_rx/brake_cmd
#/lexus3/pacmod/as_rx/headlight_cmd
#/lexus3/pacmod/as_rx/horn_cmd
#/lexus3/pacmod/as_rx/rear_pass_door_cmd
#/lexus3/pacmod/as_rx/shift_cmd
#/lexus3/pacmod/as_rx/steer_cmd
#/lexus3/pacmod/as_rx/turn_cmd
#/lexus3/pacmod/as_tx/all_system_statuses
#/lexus3/pacmod/as_tx/enabled
#/lexus3/pacmod/as_tx/vehicle_speed
#/lexus3/pacmod/can_rx
#/lexus3/pacmod/can_tx
#/lexus3/pacmod/parsed_tx/accel_aux_rpt
#/lexus3/pacmod/parsed_tx/accel_rpt
#/lexus3/pacmod/parsed_tx/brake_aux_rpt
#/lexus3/pacmod/parsed_tx/brake_rpt
#/lexus3/pacmod/parsed_tx/component_rpt
#/lexus3/pacmod/parsed_tx/date_time_rpt
#/lexus3/pacmod/parsed_tx/global_rpt
#/lexus3/pacmod/parsed_tx/headlight_aux_rpt
#/lexus3/pacmod/parsed_tx/headlight_rpt
#/lexus3/pacmod/parsed_tx/horn_rpt
#/lexus3/pacmod/parsed_tx/lat_lon_heading_rpt
#/lexus3/pacmod/parsed_tx/parking_brake_status_rpt
#/lexus3/pacmod/parsed_tx/rear_pass_door_rpt
#/lexus3/pacmod/parsed_tx/shift_aux_rpt
#/lexus3/pacmod/parsed_tx/shift_rpt
#/lexus3/pacmod/parsed_tx/steer_aux_rpt
#/lexus3/pacmod/parsed_tx/steer_rpt
#/lexus3/pacmod/parsed_tx/turn_aux_rpt
#/lexus3/pacmod/parsed_tx/turn_rpt
#/lexus3/pacmod/parsed_tx/vehicle_speed_rpt
#/lexus3/pacmod/parsed_tx/vin_rpt
#/lexus3/pacmod/parsed_tx/wheel_speed_rpt
#/lexus3/pacmod/parsed_tx/yaw_rate_rpt
#/lexus3/zed_node/atm_press
#/lexus3/zed_node/imu/data
#/lexus3/zed_node/imu/data_raw
#/lexus3/zed_node/imu/mag
#/lexus3/zed_node/left/camera_info
#/lexus3/zed_node/left/image_rect_color
#/lexus3/zed_node/left_cam_imu_transform
#/lexus3/zed_node/odom
#/lexus3/zed_node/parameter_descriptions
#/lexus3/zed_node/parameter_updates
#/lexus3/zed_node/pose
#/lexus3/zed_node/pose_with_covariance
#/lexus3/zed_node/right/camera_info
#/lexus3/zed_node/right/image_rect_color
#/lexus3/zed_node/temperature/imu
#/lexus3/zed_node/temperature/left
#/lexus3/zed_node/temperature/right
#/rosout
#/rosout_agg
#/tf
#/tf_static