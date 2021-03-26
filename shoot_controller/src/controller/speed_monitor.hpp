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
	double current_bullet_velocity;
	double update_velocity(double latest_velocity);
};
