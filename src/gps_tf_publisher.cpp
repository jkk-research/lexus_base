#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

// EN: geometry_msgs/Pose to tf

// Callback for pose messages
void vehiclePoseCallback(const geometry_msgs::PoseStamped &pos_msg){
  //ROS_INFO_STREAM(pos_msg.pose.position.x);
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped tf_map_gps;
  tf_map_gps.header.stamp = pos_msg.header.stamp;
  tf_map_gps.header.frame_id = "map";
  tf_map_gps.child_frame_id = "gps";
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
  ros::Subscriber sub_current_pose = n.subscribe(pose_topic, 1, vehiclePoseCallback);
  ROS_INFO_STREAM("Node started: " << ros::this_node::getName() << " subscribed: " << pose_topic);
  ros::spin();
  return 0;
}


/*

geometry_msgs::TransformStamped tf_world_map;
  NavMsgTfPublisher(std::shared_ptr<ros::NodeHandle> nh,
		  const std::string msg_frame_id="map"):
			  szenergy::AbstractGpsTfPublisher(nh, msg_frame_id)
  {
  }

  virtual void init() override
  {
  	  initRosPublishers();
	  sub_currentpose = nh->subscribe("/gps/current_pose", 10,
		  &NavMsgTfPublisher::CbCurrentPose, this);
  }


  void CbCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    static tf2_ros::TransformBroadcaster br;
    // We can expect, that current pose will be available
    using namespace szenergy;
    switch(internal_state)
    {
      case NavMsgTfPublisherState::START:
      {
        ref_pose.header = msg->header;
        ref_pose.pose = msg->pose;
        internal_state = NavMsgTfPublisherState::INITIALIZED;
        // Set reference TF map
        tf_world_map.header.frame_id = msg_frame_id;
        tf_world_map.child_frame_id = "map";
        tf_world_map.transform.translation.x = ref_pose.pose.position.x;
        tf_world_map.transform.translation.y = ref_pose.pose.position.y;
        tf_world_map.transform.translation.z = ref_pose.pose.position.z;
        tf2::Quaternion q;
        tf_world_map.transform.rotation.x = ref_pose.pose.orientation.x;
        tf_world_map.transform.rotation.y = ref_pose.pose.orientation.y;
        tf_world_map.transform.rotation.z = ref_pose.pose.orientation.z;
        tf_world_map.transform.rotation.w = ref_pose.pose.orientation.w;
        ROS_INFO("Initialized TF publisher from first reference message");
      }
      break;
      case NavMsgTfPublisherState::INITIALIZED:
      {
        // Propagate stamped messages
        tf_world_map.header.stamp = msg->header.stamp;
        tf_map_baselink.header.stamp = msg->header.stamp;
        // Set reference TF base_link
        // Publish current pose to Autoware localization
        msg_pose.header.stamp = msg->header.stamp;
        //msg_pose.header.stamp = ros::Time::now();
        msg_pose.pose.position.x = msg->pose.position.x;
        msg_pose.pose.position.y = msg->pose.position.y;
        msg_pose.pose.position.z = msg->pose.position.z;
        msg_pose.pose.orientation.x = msg->pose.orientation.x;
        msg_pose.pose.orientation.y = msg->pose.orientation.y;
        msg_pose.pose.orientation.z = msg->pose.orientation.z;
        msg_pose.pose.orientation.w = msg->pose.orientation.w;
        tf_map_gps.header.stamp = msg->header.stamp;
        tf_map_gps.header.frame_id = msg_frame_id;
        tf_map_gps.child_frame_id = "gps";
        tf_map_gps.transform.translation.x = msg->pose.position.x;
        tf_map_gps.transform.translation.y = msg->pose.position.y;
        tf_map_gps.transform.translation.z = msg->pose.position.z;
        tf2::Quaternion q;
        tf_map_gps.transform.rotation.x = msg->pose.orientation.x;
        tf_map_gps.transform.rotation.y = msg->pose.orientation.y;
        tf_map_gps.transform.rotation.z = msg->pose.orientation.z;
        tf_map_gps.transform.rotation.w = msg->pose.orientation.w;
        pub_current_pose.publish(msg_pose);
        // Publish transforms
        br.sendTransform(tf_map_gps);
      }
      break;
    }
  }

};


*/