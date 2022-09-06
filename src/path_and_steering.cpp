// publishes nav_msgs/Path and steering marker 

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <pacmod_msgs/SystemRptFloat.h>

double wheelbase = 2.79; // TODO
double steering_angle; 
int path_size;
bool steering_enabled;
bool first_run = true, publish_steer_marker;
const double map_gyor_0_x = 697237.0, map_gyor_0_y = 5285644.0;
const double map_zala_0_x = 639770.0, map_zala_0_y = 5195040.0;
std::string marker_color;
ros::Publisher marker_pub, path_pub;
nav_msgs::Path path;
geometry_msgs::PoseStamped actual_pose;
int location = 0;
namespace locations // add when new locations appear
{
  enum LOCATIONS
  {
    DEFAULT = 0,
    GYOR,
    ZALA,
    NOT_USED_YET
  };
}
int getLocation(const geometry_msgs::PoseStamped &pose){
    double distance_from_gyor = std::sqrt(std::pow(pose.pose.position.x - map_gyor_0_x, 2) + std::pow(pose.pose.position.y - map_gyor_0_y, 2));
    double distance_from_zala = std::sqrt(std::pow(pose.pose.position.x - map_zala_0_x, 2) + std::pow(pose.pose.position.y - map_zala_0_y, 2));
    //ROS_INFO_STREAM("distance_from_gyor: " << distance_from_gyor << " distance_from_zala: " << distance_from_zala);
    if(distance_from_gyor < 50000.0){ // 50 km from Gyor
        ROS_INFO_STREAM("Welcome to Gyor");
        return locations::GYOR;
    }
    else if(distance_from_zala < 50000.0){ // 50 km from Zalaegerszeg
        ROS_INFO_STREAM("Welcome to Zala");
        return locations::ZALA;
    }
    else{
        ROS_INFO_STREAM("Default map");
        return locations::DEFAULT;
    }
}

// Callback for steering wheel messages
void vehicleSteeringCallback(const pacmod_msgs::SystemRptFloat &steer_msg){
    steering_angle = steer_msg.output * 0.1; // TODO steering >> wheel ratio
    steering_enabled = steer_msg.enabled;
}

// Callback for pose messages
void vehiclePoseCallback(const geometry_msgs::PoseStamped &pos_msg){
    actual_pose = pos_msg;
}

void loop(){
    if (publish_steer_marker)
    {
        visualization_msgs::Marker steer_marker;
        steer_marker.header.frame_id = "base_link";
        steer_marker.header.stamp = ros::Time::now();
        steer_marker.ns = "steering_path";
        steer_marker.id = 0;
        steer_marker.type = steer_marker.LINE_STRIP;
        steer_marker.action = visualization_msgs::Marker::ADD;
        steer_marker.pose.position.x = 0;
        steer_marker.pose.position.y = 0;
        steer_marker.pose.position.z = 0;
        steer_marker.pose.orientation.x = 0.0;
        steer_marker.pose.orientation.y = 0.0;
        steer_marker.pose.orientation.z = 0.0;
        steer_marker.pose.orientation.w = 1.0;
        steer_marker.scale.x = 0.6;
        if(marker_color == "r"){
            steer_marker.color.r = 0.96f; steer_marker.color.g = 0.22f; steer_marker.color.b = 0.06f;
        }
        else if(marker_color == "b"){
            steer_marker.color.r = 0.02f; steer_marker.color.g = 0.50f; steer_marker.color.b = 0.70f;
        }
        else{ // yellow
            steer_marker.color.r = 0.94f; steer_marker.color.g = 0.83f; steer_marker.color.b = 0.07f;
        }
        steer_marker.color.a = 1.0;
        steer_marker.lifetime = ros::Duration();
        double marker_pos_x = 0.0, marker_pos_y = 0.0, theta = 0.0;
        for (int i = 0; i < 100; i++)
        {
            marker_pos_x += 0.01 * 10 * cos(theta);
            marker_pos_y += 0.01 * 10 * sin(theta);
            theta += 0.01 * 10 / wheelbase * tan(steering_angle);
            geometry_msgs::Point p;
            p.x = marker_pos_x;
            p.y = marker_pos_y;
            steer_marker.points.push_back(p);
        }
        marker_pub.publish(steer_marker);
        steer_marker.points.clear();
    }
    geometry_msgs::PoseStamped pose;
    std::string current_map = "empty";
    pose.header.stamp = ros::Time::now();
    if ((actual_pose.pose.position.x > 0.001 || actual_pose.pose.position.x < -0.001) && !isnan(actual_pose.pose.position.y) && !isinf(actual_pose.pose.position.y)){
        if (first_run){
            location = getLocation(actual_pose);
            first_run = false;
        }
        switch (location)
        {
        case locations::DEFAULT:
            pose.pose.position = actual_pose.pose.position;
            current_map = "map";
            break;
        case locations::GYOR:
            pose.pose.position.x = actual_pose.pose.position.x - map_gyor_0_x; 
            pose.pose.position.y = actual_pose.pose.position.y - map_gyor_0_y; 
            current_map = "map_gyor_0";
        break;
        case locations::ZALA:
            pose.pose.position.x = actual_pose.pose.position.x - map_zala_0_x; 
            pose.pose.position.y = actual_pose.pose.position.y - map_zala_0_y; 
            current_map = "map_zala_0";
            break;
        }
        pose.header.frame_id = current_map;
        path.header.frame_id = current_map;
        pose.pose.orientation = actual_pose.pose.orientation; 
        path.poses.push_back(pose);
        path.header.stamp = ros::Time::now();
    }
    path.poses.push_back(pose);
    // keep only the last n (path_size) path message
    if (path.poses.size() > path_size){
        int shift = path.poses.size() - path_size;
        path.poses.erase(path.poses.begin(), path.poses.begin() + shift);
    }

    path_pub.publish(path);

}

int main(int argc, char **argv)
{
    std::string pose_topic, marker_topic, path_topic;
    ros::init(argc, argv, "speed_control");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    n_private.param<std::string>("pose_topic", pose_topic, "/current_pose");
    n_private.param<std::string>("marker_topic", marker_topic, "/marker_steering");
    n_private.param<std::string>("path_topic", path_topic, "/marker_path");
    n_private.param<std::string>("marker_color", marker_color, "y");
    n_private.param<bool>("publish_steer_marker", publish_steer_marker, true);
    n_private.param<int>("path_size", path_size, 100);
    ros::Subscriber sub_steer = n.subscribe("/pacmod/parsed_tx/steer_rpt", 1, vehicleSteeringCallback);
    ros::Subscriber sub_current_pose = n.subscribe(pose_topic, 1, vehiclePoseCallback);
    marker_pub = n.advertise<visualization_msgs::Marker>(marker_topic, 1);
    path_pub = n.advertise<nav_msgs::Path>(path_topic, 1);
    ROS_INFO_STREAM("Node started: " << ros::this_node::getName() << " subscribed: " << pose_topic << " publishing: " << marker_topic << " " << path_topic);
    ros::Rate rate(20); // ROS Rate at 20Hz
    while (ros::ok()) {
        loop();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}