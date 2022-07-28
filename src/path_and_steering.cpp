// publishes nav_msgs/Path and steering marker 

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

double wheelbase = 1.685; // TODO
double steering_angle; 
int path_size;
ros::Publisher marker_pub, path_pub;
nav_msgs::Path path;

// Callback for steering wheel messages
void vehicleSteeringCallback(){
    steering_angle = 0.1; //TODO
}

// Callback for pose messages
void vehiclePoseCallback(const geometry_msgs::PoseStamped &pos_msg){
    geometry_msgs::PointStamped point;
    point.header.frame_id = "base_link";
    point.header.stamp = ros::Time::now();
    point.point.x = cos(steering_angle) * 2.;
    point.point.y = sin(steering_angle) * 2.;

    visualization_msgs::Marker pure_marker;
    pure_marker.header.frame_id = "base_link";
    pure_marker.header.stamp = ros::Time::now();
    pure_marker.ns = "steering_path";
    pure_marker.id = 0;
    pure_marker.type = pure_marker.LINE_STRIP;
    pure_marker.action = visualization_msgs::Marker::ADD;
    pure_marker.pose.position.x = 0;
    pure_marker.pose.position.y = 0;
    pure_marker.pose.position.z = 0;
    pure_marker.pose.orientation.x = 0.0;
    pure_marker.pose.orientation.y = 0.0;
    pure_marker.pose.orientation.z = 0.0;
    pure_marker.pose.orientation.w = 1.0;
    pure_marker.scale.x = 0.6;
    pure_marker.color.r = 0.94f;
    pure_marker.color.g = 0.83f;
    pure_marker.color.b = 0.07f;
    pure_marker.color.a = 1.0;
    pure_marker.lifetime = ros::Duration();
    double marker_pos_x = 0.0, marker_pos_y = 0.0, theta = 0.0;
    for (int i = 0; i < 100; i++)
    {
        marker_pos_x += 0.01 * 10 * cos(theta);
        marker_pos_y += 0.01 * 10 * sin(theta);
        theta += 0.01 * 10 / wheelbase * tan(steering_angle);
        geometry_msgs::Point p;
        p.x = marker_pos_x;
        p.y = marker_pos_y;
        pure_marker.points.push_back(p);
    }
    marker_pub.publish(pure_marker);
    pure_marker.points.clear();
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position = pos_msg.pose.position; 
    path.poses.push_back(pose);
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
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
    n_private.param<int>("path_size", path_size, 100);
    // subscribe to steering TODO vehicleSteeringCallback /pacmod/parsed_tx/steer_rpt.command SystemRptFloat
    ros::Subscriber sub_current_pose = n.subscribe(pose_topic, 1, vehiclePoseCallback);
    marker_pub = n.advertise<visualization_msgs::Marker>(marker_topic, 1);
    path_pub = n.advertise<nav_msgs::Path>(path_topic, 1);
    ROS_INFO_STREAM("Node started: " << ros::this_node::getName() << " subscribed: " << pose_topic << " publishing: " << marker_topic << " " << path_topic);
    ros::spin();
    return 0;
}