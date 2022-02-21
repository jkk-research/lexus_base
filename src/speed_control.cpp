#include <iostream>
#include <vector>
#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include <lexus_base/MyParamsConfig.h>
#include "pacmod_msgs/VehicleSpeedRpt.h"
#include "pacmod_msgs/SystemRptFloat.h"
#include "geometry_msgs/Twist.h"

ros::Publisher accel_pub;
ros::Publisher brake_pub;
double vehicle_speed_actual;
double vehicle_speed_reference;
double speed_diff;
pacmod_msgs::SystemRptFloat accel_command;
pacmod_msgs::SystemRptFloat brake_command;
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
    accel_pub = n.advertise<pacmod_msgs::SystemRptFloat>("/temp/pacmod/parsed_tx/accel_rpt", 1); // TODO
    brake_pub = n.advertise<pacmod_msgs::SystemRptFloat>("/temp/pacmod/parsed_tx/brake_rpt", 1); // TODO
    ros::spin();
    return 0;
}

// Callback for speed messages
void speedCurrentCallback(const pacmod_msgs::VehicleSpeedRpt &speed_msg)
{
    vehicle_speed_actual = speed_msg.vehicle_speed;
    //ROS_INFO_STREAM("speed: " << speed_msg.vehicle_speed);
    speed_diff = vehicle_speed_reference - vehicle_speed_actual ;
    //ROS_INFO_STREAM(" diff: " << speed_diff);
    if(speed_diff > 0){
        //ROS_INFO_STREAM("accelerate");
        accel_command.command = speed_diff * pid.p_gain_accel;
        brake_command.command = 0.0;
    }
    else if(speed_diff < 0){
        //ROS_INFO_STREAM("brake");
        brake_command.command = speed_diff * pid.p_gain_brake;
        accel_command.command = 0.0;
    }
    accel_pub.publish(accel_command);
    brake_pub.publish(brake_command);
}

void speedReferenceCallback(const geometry_msgs::Twist &ref_msg)
{
    vehicle_speed_reference = ref_msg.linear.x;
}

// Parameter callback
void paramsCallback(my_dyn_rec::MyParamsConfig &config, uint32_t level)
{
    pid = config;
    ROS_INFO_STREAM("p: " << config.p_gain_accel << " | i: " << config.i_gain_accel << "|| p: " << config.p_gain_brake << " | i: " << config.i_gain_brake  );
}