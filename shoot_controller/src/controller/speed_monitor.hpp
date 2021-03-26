#pragma once

#include <ros/ros.h>
#include <vector>

class SpeedMonitor {
  public:
	SpeedMonitor();
	double get_current_bullet_velocity() const;

  private:
	ros::Subscriber robot_shoot_sub;
	ros::Publisher current_bullet_velocity_pub;
	bool use_measured_velocity = true;
	double current_bullet_velocity = 8.0;
	double update_velocity(double latest_velocity);
};
