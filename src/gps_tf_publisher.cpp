#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

std::string child_frame_id, header_frame_id;
// EN: geometry_msgs/Pose to tf

// Callback for pose messages
void vehiclePoseCallback(const geometry_msgs::PoseStamped &pos_msg){
  //ROS_INFO_STREAM(pos_msg.pose.position.x);
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped tf_map_gps;
  tf_map_gps.header.stamp = pos_msg.header.stamp;
  tf_map_gps.header.frame_id = header_frame_id;
  tf_map_gps.child_frame_id = child_frame_id;
  tf_map_gps.transform.translation.x = pos_msg.pose.position.x;
  tf_map_gps.transform.translation.y = pos_msg.pose.position.y;
  tf_map_gps.transform.translation.z = pos_msg.pose.position.z;
  tf_map_gps.transform.rotation.x = pos_msg.pose.orientation.x;
  tf_map_gps.transform.rotation.y = pos_msg.pose.orientation.y;
  tf_map_gps.transform.rotation.z = pos_msg.pose.orientation.z;
  tf_map_gps.transform.rotation.w = pos_msg.pose.orientation.w;
  // Publish transforms
  br.sendTransform(tf_map_gps);
}


int main(int argc, char **argv)
{
  std::string pose_topic;
  ros::init(argc, argv, "lexus_nav_msg_tf_pub");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  n_private.param<std::string>("pose_topic", pose_topic, "/gps/duro/current_pose");
  n_private.param<std::string>("header_frame_id", header_frame_id, "map");
  n_private.param<std::string>("child_frame_id", child_frame_id, "gps");
  ros::Subscriber sub_current_pose = n.subscribe(pose_topic, 1, vehiclePoseCallback);
  ROS_INFO_STREAM("Node started: " << ros::this_node::getName() << " subscribed: " << pose_topic << " child_frame_id: " << child_frame_id);
  ros::spin();
  return 0;
}
