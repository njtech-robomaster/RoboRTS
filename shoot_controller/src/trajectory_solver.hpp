#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class TrajectorySolver {
  public:
	TrajectorySolver();
	bool publish_simulation();

	std::string base_frame = "base_link";
	std::string shoot_frame = "shoot_link";
	double bullet_velocity = 8.0;
	double sim_time_step = 0.01;

  private:
	ros::Publisher trajectory_path_pub;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;
};
