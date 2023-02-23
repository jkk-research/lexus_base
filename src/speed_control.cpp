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
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"


ros::Publisher accel_pub;
ros::Publisher brake_pub;
ros::Publisher steer_pub;
ros::Publisher enable_pub;
ros::Publisher status_string_pub;
double vehicle_speed_actual, vehicle_speed_reference, vehicle_steering_reference;
double speed_diff, speed_diff_kmh, speed_diff_prev;
ros::Time current_time, prev_time;
double dt = 0.033333; // /lexus3/pacmod/parsed_tx/vehicle_speed_rpt is assumed 30 Hz
double p_out_accel, i_out_accel, d_out_accel = 0.0;
double p_out_brake, i_out_brake, d_out_brake = 0.0;
double const _max_i_accel = 1.0, _max_i_brake = 1.0; // anti windup constants TODO: check valid params 
double t_integ_accel, t_integ_brake; // anti windup
double t_derivative_accel, t_derivative_brake;
double accel_command_prev = 0.0, brake_command_prev = 0.0;
bool autonom_status_changed = true;
bool first_run = true;
bool control_state = true;
std_msgs::String status_string_msg;
pacmod_msgs::SystemCmdFloat accel_command;
pacmod_msgs::SystemCmdFloat brake_command;
pacmod_msgs::SteerSystemCmd steer_command;
std_msgs::Bool enable_msg;
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
    enable_pub = n.advertise<std_msgs::Bool>("/pacmod/as_rx/enable", 1);
    status_string_pub = n.advertise<std_msgs::String>("control_status", 1);
    ros::spin();
    return 0;
}

// Callback for speed messages
void speedCurrentCallback(const pacmod_msgs::VehicleSpeedRpt &speed_msg)
{
    current_time = ros::Time::now();
    vehicle_speed_actual = speed_msg.vehicle_speed;
    steer_command.command = vehicle_steering_reference;
    //ROS_INFO_STREAM("speed: " << speed_msg.vehicle_speed);
    speed_diff = vehicle_speed_reference - vehicle_speed_actual;
    speed_diff_kmh = speed_diff * 3.6;
    //ROS_INFO_STREAM(" diff km/h: " << speed_diff_kmh);
    if( (speed_diff > -0.15 && control_state) || (speed_diff > 0.15) ){
        control_state = true;
        //ROS_INFO_STREAM("accelerate");
        p_out_accel = speed_diff * pid.p_gain_accel;
        t_integ_accel += speed_diff * dt;
        // Restrict to max Anti-Windup
        //if( t_integ_accel > _max_i_accel)
        //    t_integ_accel = _max_i_accel;
        i_out_accel = t_integ_accel * pid.i_gain_accel;
        t_derivative_accel = (speed_diff - speed_diff_prev) / dt;
        d_out_accel = pid.d_gain_accel * t_derivative_accel;
        double acclel_command_raw = p_out_accel + i_out_accel + d_out_accel;
        if (acclel_command_raw > 1.0)
        {
            t_integ_accel = acclel_command_raw - p_out_accel - d_out_accel;
            acclel_command_raw = 1.0;
        }
        accel_command.command = acclel_command_raw;
        brake_command.command = 0.0;
        // gradient limit
        if((accel_command.command - accel_command_prev) / dt > 0.4 && dt > 0.0){
            accel_command.command = accel_command_prev + 0.4 * dt;
        }
        else if((accel_command.command - accel_command_prev) / dt < -1.6 && dt > 0.0){
            accel_command.command = accel_command_prev - 1.6 * dt;
        } 
    }
    else if( (speed_diff < 0.15 && !control_state) || (speed_diff < -0.15) ){
        control_state = false;
        //ROS_INFO_STREAM("brake");
        p_out_brake = speed_diff * pid.p_gain_brake ;
        t_integ_brake += speed_diff * dt;
        // Restrict to max Anti-Windup
        if( t_integ_brake > _max_i_brake)
            t_integ_brake = _max_i_brake;
        i_out_brake = t_integ_brake * pid.i_gain_brake;
        t_derivative_brake = (speed_diff - speed_diff_prev) / dt;
        d_out_brake = pid.d_gain_brake * t_derivative_brake;
        brake_command.command = -1.0 * (p_out_brake + i_out_brake + d_out_brake);
        accel_command.command = 0.0;
        // gradient limit
        if((brake_command.command - brake_command_prev) / dt > 0.8 && dt > 0.0){
            brake_command.command = brake_command_prev + 0.8 * dt;
        }
    }
    if(autonom_status_changed){ 
        accel_command.clear_override = true;
        brake_command.clear_override = true;
        steer_command.clear_override = true;
        autonom_status_changed = false; // just once
    }
    else{
        accel_command.clear_override = false;
        brake_command.clear_override = false;
        steer_command.clear_override = false;
    }    
    if(!first_run){
        dt = (current_time - prev_time).toSec();
        status_string_msg.data = "lateral_control_started";
        //ROS_INFO_STREAM("dt: " << dt);
    }
    {
        // /lexus3/pacmod/parsed_tx/vehicle_speed_rpt is assumed 30 Hz
        dt = 0.033333;
    }


    steer_command.rotation_rate = 3.3;
    accel_command.header.frame_id = "pacmod";
    accel_command.enable = true;
    brake_command.enable = true;
    steer_command.enable = true;
    enable_msg.data = true;
    enable_pub.publish(enable_msg);
    accel_pub.publish(accel_command);
    brake_pub.publish(brake_command);
    steer_pub.publish(steer_command);
    status_string_msg.data = control_state ? "accel" : "brake";
    if(status_string_msg.data.compare(""))
        status_string_pub.publish(status_string_msg);
    status_string_msg.data = "";
    prev_time = current_time; 
    accel_command_prev = accel_command.command;
    brake_command_prev = brake_command.command;
    speed_diff_prev = speed_diff;
    
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