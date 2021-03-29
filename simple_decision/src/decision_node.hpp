#pragma once

#include "chassis_rotation.hpp"
#include "geometry_msgs/Pose.h"
#include "goal_executor.hpp"
#include <optional>
#include <ros/ros.h>

class DecisionNode {
  public:
	DecisionNode();

  private:
	ChassisRotation self_rotation;
	GoalExecutor goal_executor;

	geometry_msgs::Pose boot_area;
	int bullet_area;
	int hp_area;

	ros::Subscriber game_status_sub;
	ros::Subscriber game_zone_sub;
	ros::Publisher yaw_focus_pub;
	ros::Timer control_loop_timer;
	bool in_play;

	enum State { STAY_HOME, GO_OUT };
	State state;

	void control_loop();
};
