#pragma once

#include "speed_monitor.hpp"
#include <geometry_msgs/PointStamped.h>
#include <roborts_msgs/YawFocus.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class ShootController {
  public:
	ShootController();

  private:
	bool aim_and_shoot(const geometry_msgs::PointStamped &target);
	bool track(const geometry_msgs::PointStamped &target);
	bool track_yaw_focus(const geometry_msgs::PointStamped &target);
	bool shoot();
	void track_moving_reference(bool control_gimbal);
	void switch_moving_reference(bool control_gimbal);
	void control_loop();

	std::string fixed_frame = "odom";
	std::string yaw_base_frame = "yaw_base_link";
	std::string pitch_base_frame = "pitch_base_link";
	double gun_barrel_length = 0.0;

	bool has_last_target;
	geometry_msgs::PointStamped last_target;

	bool in_play;
	int heat;
	int heat_cooling_rate;
	int heat_cooling_limit;

	bool has_moving_reference;
	geometry_msgs::PointStamped moving_reference;

	roborts_msgs::YawFocus yaw_focus;

	SpeedMonitor speed_monitor;
	ros::Publisher gimbal_pub;
	ros::Subscriber armor_target_sub;
	ros::Subscriber game_status_sub;
	ros::Subscriber robot_heat_sub;
	ros::Subscriber robot_status_sub;
	ros::Subscriber yaw_focus_sub;
	ros::ServiceClient cmd_shoot;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;
	ros::Timer control_loop_timer;
};
