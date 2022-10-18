#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

// EN: /current_pose topic (geometry_msgs/Pose) from tf

int main(int argc, char **argv)
{
  std::string pose_topic;
  ros::init(argc, argv, "lexus_current_pose_pub");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  n_private.param<std::string>("pose_topic", pose_topic, "/current_pose");
  ros::Publisher curr_pub = n.advertise<geometry_msgs::PoseStamped>(pose_topic, 1);
  ROS_INFO_STREAM("Node started: " << ros::this_node::getName() << " publishing: " << pose_topic);


  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(20.0);
  while (n.ok()){
    geometry_msgs::TransformStamped transform_stamped;
    try{
      transform_stamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
      geometry_msgs::PoseStamped curr_pose;
      curr_pose.header.stamp = transform_stamped.header.stamp;
      curr_pose.header.frame_id = "map";
      //ROS_INFO_STREAM(transform_stamped.transform.translation.x);
      curr_pose.pose.position.x = transform_stamped.transform.translation.x;
      curr_pose.pose.position.y = transform_stamped.transform.translation.y;
      curr_pose.pose.position.z = transform_stamped.transform.translation.z;
      curr_pose.pose.orientation = transform_stamped.transform.rotation;
      curr_pub.publish(curr_pose);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(0.1).sleep();
      continue;
    }
    rate.sleep();
  }
  return 0;
}
