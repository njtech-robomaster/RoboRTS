#pragma once

#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class TrajectorySolver {
  public:
	TrajectorySolver(std::function<double()> bullet_velocity_fn);

  private:
	bool aim_target(const geometry_msgs::PointStamped &target);

	std::string yaw_base_frame = "yaw_base_link";
	std::string pitch_base_frame = "pitch_base_link";
	std::string shoot_frame = "shoot_link";

	std::function<double()> bullet_velocity_fn;
	ros::Publisher gimbal_pub;
	ros::Subscriber armor_target_sub;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;
};
