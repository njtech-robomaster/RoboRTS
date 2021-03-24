#pragma once

#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class TrajectorySolver {
  public:
	TrajectorySolver();
	bool aim_target(const geometry_msgs::PointStamped &target);

	std::string yaw_base_frame = "yaw_base_link";
	std::string pitch_base_frame = "pitch_base_link";
	std::string shoot_frame = "shoot_link";
	double bullet_velocity = 8.0;

  private:
	ros::Publisher gimbal_pub;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;
};
