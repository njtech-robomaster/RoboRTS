#include "decision_node.hpp"
#include "goals.hpp"
#include <roborts_msgs/GameStatus.h>
#include <roborts_msgs/GameZoneArray.h>
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
    : boot_area{get_boot_area()},
      bullet_area{-1}, hp_area{-1}, in_play{false}, state{STAY_HOME} {
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
}

void DecisionNode::control_loop() {
	int robot_id = 0;
	if (!ros::param::getCached("robot_id", robot_id)) {
		ROS_WARN("robot_id param not found");
	}

	if (state == STAY_HOME) {
		int area_to_go = -1;
		if (robot_id == 1) {
			if (bullet_area != -1) {
				ROS_INFO("Go to bullet area");
				area_to_go = bullet_area;
			} else if (hp_area != -1) {
				ROS_INFO("Go to hp area");
				area_to_go = hp_area;
			}
		} else if (robot_id == 2) {
			// stay home
		}

		roborts_msgs::YawFocus yaw_focus;
		yaw_focus.focus_point.header.frame_id = "map";
		yaw_focus.focus_point.header.stamp = ros::Time{0};

		if (area_to_go == -1) {
			self_rotation.start();
			yaw_focus.has_focus = true;
		} else {
			self_rotation.stop();
			yaw_focus.has_focus = false;
			state = GO_OUT;
			goal_executor.goto_goal(
			    get_buff_zone_location(area_to_go), [this](bool) {
				    // go back home
				    ROS_INFO("Arrived buff area");
				    goal_executor.goto_goal(boot_area, [this](bool) {
					    // TODO if not succeeded?
					    state = STAY_HOME;
					    ROS_INFO("Arrived home");
				    });
			    });
		}
		yaw_focus_pub.publish(yaw_focus);
	}
}
