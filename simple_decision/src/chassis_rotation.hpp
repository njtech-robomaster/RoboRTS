#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class ChassisRotation {
  public:
	ChassisRotation();

	double angle_min = -M_PI / 3;
	double angle_max = M_PI / 3;
	double angle_speed = 2.0;

	void start();
	void stop();

  private:
	ros::Subscriber odom_sub;
	ros::Publisher cmd_vel_pub;
	double angle_speed_sig = 1;
	double angle_speed_abs = 1;

	enum State { INACTIVE, INITIALIZING, ACTIVE };
	State state = INACTIVE;
	double initial_angle;
};
