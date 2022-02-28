#include <iostream>
#include <vector>
#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include <lexus_base/MyParamsConfig.h>
#include "pacmod_msgs/VehicleSpeedRpt.h"
#include "pacmod_msgs/SystemCmdFloat.h"
#include "pacmod_msgs/SystemRptFloat.h"
#include "pacmod_msgs/SteerSystemCmd.h"
#include "geometry_msgs/Twist.h"


ros::Publisher accel_pub;
ros::Publisher brake_pub;
ros::Publisher steer_pub;
double vehicle_speed_actual;
double vehicle_speed_reference;
double vehicle_steering_reference;
double speed_diff, speed_diff_kmh;
pacmod_msgs::SystemCmdFloat accel_command;
pacmod_msgs::SystemCmdFloat brake_command;
pacmod_msgs::SteerSystemCmd steer_command;
my_dyn_rec::MyParamsConfig pid;

// Functions
void speedCurrentCallback(const pacmod_msgs::VehicleSpeedRpt &speed_msg);
void speedReferenceCallback(const geometry_msgs::Twist &ref_msg);
void paramsCallback(my_dyn_rec::MyParamsConfig &config, uint32_t level);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_control");
    ROS_INFO_STREAM("Node started: " << ros::this_node::getName());
    dynamic_reconfigure::Server<my_dyn_rec::MyParamsConfig> server;
    dynamic_reconfigure::Server<my_dyn_rec::MyParamsConfig>::CallbackType f;
    f = boost::bind(&paramsCallback, _1, _2);
    server.setCallback(f);

    ros::NodeHandle n;
    ros::Subscriber sub_current_speeed = n.subscribe("/pacmod/parsed_tx/vehicle_speed_rpt", 1, speedCurrentCallback);
    ros::Subscriber sub_reference_speed = n.subscribe("/cmd_vel", 1, speedReferenceCallback);
    accel_pub = n.advertise<pacmod_msgs::SystemCmdFloat>("/pacmod/as_rx/accel_cmd", 1);
    brake_pub = n.advertise<pacmod_msgs::SystemCmdFloat>("/pacmod/as_rx/brake_cmd", 1);
    steer_pub = n.advertise<pacmod_msgs::SteerSystemCmd>("/pacmod/as_rx/steer_cmd", 1);
    ros::spin();
    return 0;
}

// Callback for speed messages
void speedCurrentCallback(const pacmod_msgs::VehicleSpeedRpt &speed_msg)
{
    vehicle_speed_actual = speed_msg.vehicle_speed;
    steer_command.command = vehicle_steering_reference;
    //ROS_INFO_STREAM("speed: " << speed_msg.vehicle_speed);
    speed_diff = vehicle_speed_reference - vehicle_speed_actual;
    speed_diff_kmh = speed_diff * 3.6;
    //ROS_INFO_STREAM(" diff km/h: " << speed_diff_kmh);
    if(speed_diff > 0){
        //ROS_INFO_STREAM("accelerate");
        accel_command.command = speed_diff * pid.p_gain_accel;
        brake_command.command = 0.0;
    }
    else if(speed_diff < 0){
        //ROS_INFO_STREAM("brake");
        brake_command.command = -1.0 * speed_diff * pid.p_gain_brake;
        accel_command.command = 0.0;
    }
    steer_command.rotation_rate = 3.3;
    accel_command.header.frame_id = "pacmod";
    accel_command.enable = true;
    brake_command.enable = true;
    steer_command.enable = true;
    accel_pub.publish(accel_command);
    brake_pub.publish(brake_command);
    steer_pub.publish(steer_command);
    
}

void speedReferenceCallback(const geometry_msgs::Twist &ref_msg)
{
    vehicle_speed_reference = ref_msg.linear.x;
    vehicle_steering_reference = ref_msg.angular.z;
}

// Parameter callback
void paramsCallback(my_dyn_rec::MyParamsConfig &config, uint32_t level)
{
    pid = config;
    ROS_INFO_STREAM("p: " << config.p_gain_accel << " | i: " << config.i_gain_accel << "|| p: " << config.p_gain_brake << " | i: " << config.i_gain_brake  );
}