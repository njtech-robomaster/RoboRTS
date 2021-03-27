#pragma once

#include <geometry_msgs/PointStamped.h>
#include "speed_monitor.hpp"
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class ShootController {
  public:
	ShootController();

  private:
	bool aim_target(const geometry_msgs::PointStamped &target);
	void control_loop();

	std::string fixed_frame = "odom";
	std::string yaw_base_frame = "yaw_base_link";
	std::string pitch_base_frame = "pitch_base_link";
	std::string shoot_frame = "shoot_link";

	bool has_last_target;
	geometry_msgs::PointStamped last_target;

	SpeedMonitor speed_monitor;
	ros::Publisher gimbal_pub;
	ros::Subscriber armor_target_sub;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;
	ros::Timer control_loop_timer;
};
