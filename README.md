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

# Install CAN pacmod

- https://github.com/astuff/kvaser_interface
- https://github.com/astuff/pacmod3
- https://github.com/astuff/pacmod_game_control

``` c
sudo apt install ros-melodic-socketcan-interface
sudo apt install ros-melodic-socketcan-bridge
sudo apt install can-utils 
```

``` c
modprobe can_dev
modprobe can
modprobe can_raw
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

sudo ip link set can0 up type can bitrate 500000
```
## Test

``` r
rosrun kvaser_interface list_channels 
candump can0
cansniffer can0
```
## Config

``` c
roscd pacmod_game_control/launch/
code pacmod_game_control.launch 
```
``` xml
<arg name="use_socketcan" default="true" />
```
`roslaunch pacmod_game_control pacmod_game_control.launch`

## Further reading
- https://autonomoustuff.atlassian.net/wiki/spaces/RW/overview
- https://github.com/astuff
- http://wiki.ros.org/pacmod3

# Rosgraph

![](img/rosgraph_before01.svg)