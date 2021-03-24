#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class TrajectoryPublisher {
  public:
	TrajectoryPublisher();
	bool publish();

  private:
	ros::Publisher path_publisher;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;
    std::string base_frame;
    std::string shoot_frame;
};
