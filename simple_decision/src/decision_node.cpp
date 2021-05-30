#include "decision_node.hpp"
#include "goals.hpp"
#include <roborts_msgs/GameRobotHP.h>
#include <roborts_msgs/GameStatus.h>
#include <roborts_msgs/GameZoneArray.h>
#include <roborts_msgs/RobotStatus.h>
#include <roborts_msgs/YawFocus.h>

geometry_msgs::Pose get_boot_area() {
	std::string name;
	ros::param::get("~boot_area", name);
	if (name == "c1") {
		return Goals::C1;
	} else if (name == "c2") {
		return Goals::C2;
	} else if (name == "c3") {
		return Goals::C3;
	} else if (name == "c4") {
		return Goals::C4;
	}
	throw std::invalid_argument("boot_area not set");
}

geometry_msgs::Point get_buff_zone_location(int id) {
	switch (id) {
	case 0:
		return Goals::F1;
	case 1:
		return Goals::F2;
	case 2:
		return Goals::F3;
	case 3:
		return Goals::F4;
	case 4:
		return Goals::F5;
	case 5:
		return Goals::F6;
	default:
		throw std::invalid_argument("invalid buff zone id: " + id);
	}
}

DecisionNode::DecisionNode()
    : boot_area{get_boot_area()}, bullet_area{-1}, hp_area{-1}, in_play{false},
      need_hp_buff{false}, state{INIT}, is_another_dead{false}, can_attack{
                                                                    false} {
	ros::NodeHandle nh;

	game_zone_sub = nh.subscribe<roborts_msgs::GameZoneArray>(
	    "game_zone_array_status", 1,
	    [this](const roborts_msgs::GameZoneArray::ConstPtr &msg) {
		    std::string team_color;
		    ros::param::getCached("team_color", team_color);

		    uint8_t bullet_area_type;
		    uint8_t hp_area_type;
		    if (team_color == "red") {
			    bullet_area_type = roborts_msgs::GameZone::RED_BULLET_SUPPLY;
			    hp_area_type = roborts_msgs::GameZone::RED_HP_RECOVERY;
		    } else if (team_color == "blue") {
			    bullet_area_type = roborts_msgs::GameZone::BLUE_BULLET_SUPPLY;
			    hp_area_type = roborts_msgs::GameZone::BLUE_HP_RECOVERY;
		    } else {
			    ROS_WARN_STREAM("Unrecognized team_color: " << team_color);
			    return;
		    }

		    bullet_area = -1;
		    hp_area = -1;
		    for (int i = 0; i < 6; i++) {
			    roborts_msgs::GameZone zone = msg->zone[i];
			    if (zone.active) {
				    if (zone.type == bullet_area_type) {
					    bullet_area = i;
				    } else if (zone.type == hp_area_type) {
					    hp_area = i;
				    }
			    }
		    }
	    });

	game_status_sub = nh.subscribe<roborts_msgs::GameStatus>(
	    "/game_status", 10,
	    [this](const roborts_msgs::GameStatus::ConstPtr &msg) {
		    in_play = msg->game_status == roborts_msgs::GameStatus::GAME;
		    if (!in_play) {
			    self_rotation.stop();
		    }
	    });

	control_loop_timer =
	    nh.createTimer(ros::Duration{0.1}, [this](const ros::TimerEvent &) {
		    if (in_play) {
			    control_loop();
		    }
	    });

	yaw_focus_pub = nh.advertise<roborts_msgs::YawFocus>("cmd_yaw_focus", 1);

	game_robot_hp_sub = nh.subscribe<roborts_msgs::GameRobotHP>(
	    "game_robot_hp", 1,
	    [this](const roborts_msgs::GameRobotHP::ConstPtr &msg) {
		    int robot1_hp;
		    int robot2_hp;

		    std::string team_color;
		    ros::param::getCached("team_color", team_color);
		    if (team_color == "red") {
			    robot1_hp = msg->red1;
			    robot2_hp = msg->red2;
		    } else if (team_color == "blue") {
			    robot1_hp = msg->blue1;
			    robot2_hp = msg->blue2;
		    } else {
			    ROS_WARN_STREAM("Unrecognized team_color: " << team_color);
			    return;
		    }

		    if (robot1_hp == 0) {
			    std::swap(robot1_hp, robot2_hp);
		    }

		    need_hp_buff = false;
		    if (robot2_hp == 0) {
			    if (robot1_hp < 1800) {
				    need_hp_buff = true;
			    }
			    is_another_dead = true;
		    } else {
			    if ((robot1_hp < 1800 && robot2_hp < 1800) ||
			        std::min(robot1_hp, robot2_hp) < 1000) {
				    need_hp_buff = true;
			    }
			    is_another_dead = false;
		    }
	    });

	robot_status_sub = nh.subscribe<roborts_msgs::RobotStatus>(
	    "robot_status", 1,
	    [this](const roborts_msgs::RobotStatus::ConstPtr &msg) {
		    can_attack = msg->shooter_enable;
	    });
}

void DecisionNode::go_back_home() {
	ROS_INFO("Going back home");
	state = GOING_BACK;
	goal_executor.goto_goal(boot_area, [this](bool success) {
		if (success) {
			ROS_INFO("Arrived home!");
			state = STAY_HOME;
		} else {
			ROS_INFO("Couldn't go back home, try again!");
			go_back_home();
		}
	});
}

bool DecisionNode::try_goto_buff_zone() {
	int robot_id = 0;
	if (!ros::param::getCached("robot_id", robot_id)) {
		ROS_WARN("robot_id param not found");
		return false;
	}

	int area_to_go = -1;
	if (robot_id == 1 || is_another_dead) {
		if (bullet_area != -1) {
			ROS_INFO("Go to bullet area");
			area_to_go = bullet_area;
		} else if (hp_area != -1 && need_hp_buff) {
			ROS_INFO("Go to hp area");
			area_to_go = hp_area;
		}
	} else if (robot_id == 2) {
		// stay home
	}

	if (area_to_go == -1) {
		return false;
	}

	ROS_INFO("Going to buff zone F%d", area_to_go + 1);

	state = GOING_OUT;
	goal_executor.goto_goal(get_buff_zone_location(area_to_go), [this](bool) {
		ROS_INFO("Arrived buff area");
		go_back_home();
	});

	return true;
}

void DecisionNode::publish_focus_center(bool has_focus) {
	roborts_msgs::YawFocus yaw_focus;
	yaw_focus.focus_point.header.frame_id = "map";
	yaw_focus.focus_point.header.stamp = ros::Time{0};
	yaw_focus.focus_point.point.x = 0;
	yaw_focus.focus_point.point.y = 0;
	yaw_focus.focus_point.point.z = 0;
	yaw_focus.has_focus = has_focus;
	yaw_focus_pub.publish(yaw_focus);
}

void DecisionNode::control_loop() {
	bool focus_map_center = false;

	if (state == INIT) {
		go_back_home();
	} else if (state == STAY_HOME) {
		bool go_out = try_goto_buff_zone();
		if (!go_out) {
			focus_map_center = true;
		}
		if (!go_out && !can_attack) {
			self_rotation.start();
		} else {
			if (self_rotation.stop()) {
				go_back_home();
			}
		}
	} else if (state == GOING_OUT) {
	} else if (state == GOING_BACK) {
		try_goto_buff_zone();
	}

	publish_focus_center(focus_map_center);
}
