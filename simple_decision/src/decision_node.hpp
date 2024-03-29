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

	std::string boot_area;
	int bullet_area;
	int hp_area;

	ros::Subscriber game_status_sub;
	ros::Subscriber game_zone_sub;
	ros::Subscriber game_robot_hp_sub;
	ros::Subscriber robot_status_sub;
	ros::Publisher yaw_focus_pub;
	ros::Timer control_loop_timer;
	bool in_play;
	bool need_hp_buff;
	bool can_attack;
	bool is_another_dead;
	int go_back_home_try_count;

	enum State { INIT, STAY_HOME, GOING_OUT, GOING_BACK };
	State state;

	void control_loop();
	void go_back_home();
	bool try_goto_buff_zone();
	void publish_focus_center(bool has_focus);
	geometry_msgs::Pose get_buff_zone_location(int id);

	void switch_home();
};
