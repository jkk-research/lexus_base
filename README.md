# lexus_base
ROS package for basic functions on Lexus rx450h.

- https://github.com/astuff/kvaser_interface
- https://github.com/astuff/pacmod3

```
sudo apt install ros-melodic-pacmod-msgs
pip install pyqtgraph numpy scipy matplotlib
```


``` r
cd catkin_ws
git clone https://github.com/astuff/astuff_sensor_msgs
cd astuff_sensor_msgs
git checkout 25b72c1a4567f3e71c187e19c192f1e7a8a3b75e
catkin build pacmod_msgs
```


``` c
rostopic type /pacmod/parsed_tx/vehicle_speed_rpt
pacmod_msgs/VehicleSpeedRpt

rostopic type /pacmod/parsed_tx/accel_rpt
pacmod_msgs/SystemRptFloat

rostopic type /pacmod/parsed_tx/brake_rpt
pacmod_msgs/SystemRptFloat

rosmsg show pacmod_msgs/SystemRptFloat
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
bool enabled
bool override_active
bool command_output_fault
bool input_output_fault
bool output_reported_fault
bool pacmod_fault
bool vehicle_fault
float64 manual_input
float64 command
float64 output
```

# Topics

- `/pacmod/as_rx/accel_cmd` - reference
- `/pacmod/parsed_tx/accel_rpt` - actual
- `/pacmod/parsed_tx/vehicle_speed_rpt` - actual speed