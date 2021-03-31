#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class ChassisRotation {
  public:
	ChassisRotation();

	double angle_min = -M_PI / 3;
	double angle_max = M_PI / 6;
	double angle_speed = 1.8;

	bool start();
	bool stop();

  private:
	ros::Subscriber odom_sub;
	ros::Publisher cmd_vel_pub;
	double angle_speed_ctrl = 0;

	enum State { INACTIVE, INITIALIZING, ACTIVE };
	State state = INACTIVE;
	double initial_angle;
};
