#include "decision_node.hpp"
#include "goals.hpp"
#include <optional>
#include <roborts_msgs/GameRobotHP.h>
#include <roborts_msgs/GameStatus.h>
#include <roborts_msgs/GameZoneArray.h>
#include <roborts_msgs/RobotStatus.h>
#include <roborts_msgs/YawFocus.h>

geometry_msgs::Pose get_boot_area(const std::string &boot_area_name) {
	if (boot_area_name == "c1") {
		return Goals::C1;
	} else if (boot_area_name == "c2") {
		return Goals::C2;
	} else if (boot_area_name == "c3") {
		return Goals::C3;
	} else if (boot_area_name == "c4") {
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
		throw std::invalid_argument("invalid buff zone id: " +
		                            std::to_string(id));
	}
}

DecisionNode::DecisionNode()
    : bullet_area{-1}, hp_area{-1}, in_play{false}, need_hp_buff{false},
      can_attack{false}, is_another_dead{false}, state{INIT} {
	ros::NodeHandle nh;

	if (!ros::param::get("~boot_area", boot_area)) {
		throw std::invalid_argument("boot_area not set");
	}

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
	goal_executor.goto_goal(get_boot_area(boot_area), [this](bool success) {
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

enum class FieldPart { LEFT, RIGHT };

std::optional<FieldPart> check_part(double x) {
	if (x < -0.5) {
		return FieldPart::LEFT;
	} else if (x > 0.5) {
		return FieldPart::RIGHT;
	} else {
		return std::nullopt;
	}
}

bool DecisionNode::is_being_besieged() {
	auto my_color = get_my_vehicle_color();
	if (!my_color.has_value()) {
		return false;
	}

	geometry_msgs::PointStamped p0;
	p0.point.x = 0;
	p0.point.y = 0;
	p0.point.z = 0;
	p0.header.frame_id = "base_link";
	p0.header.stamp = ros::Time{0};
	geometry_msgs::PointStamped my_pos;
	try {
		tf_buffer.transform(p0, "map");
	} catch (const std::exception &e) {
		ROS_WARN("Couldn't lookup transform (is_being_besieged): %s", e.what());
		return false;
	}

	auto my_field_part = check_part(my_pos.point.x);
	if (!my_field_part.has_value()) {
		return false;
	}

	int enemy_count_in_my_part = 0;
	std::vector<Vehicle> enemy_cars;
	for (auto &car : lookout.get_cars()) {
		if (car.color != VehicleColor::UNKNOWN &&
		    car.color != my_color.value()) {
			auto their_part = check_part(car.x);
			if (their_part.has_value() && their_part.value() == my_field_part) {
				enemy_count_in_my_part++;
				enemy_cars.push_back(car);
			}
		}
	}

	if (enemy_count_in_my_part > 2) {
		auto &car1 = enemy_cars[0];
		auto &car2 = enemy_cars[1];
		auto their_distance = std::sqrt(std::pow(car1.x - car2.x, 2) +
		                                std::pow(car1.y - car2.y, 2));
		if (their_distance < 2.5) {
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	}
}

bool DecisionNode::try_run_away() {
	std::string new_home;
	if (boot_area == "c1") {
		new_home = "c4";
	} else if (boot_area == "c2") {
		new_home = "c3";
	} else if (boot_area == "c3") {
		new_home = "c2";
	} else if (boot_area == "c4") {
		new_home = "c1";
	}
	ROS_INFO("Run away from %s to %s", boot_area.c_str(), new_home.c_str());

	boot_area = new_home;
	go_back_home();
	return true;
}

void DecisionNode::control_loop() {
	bool being_besieged = false;
	if (is_being_besieged()) {
		besieged_count++;
		if (besieged_count == 30) {
			besieged_count = 0;
			being_besieged = true;
		}
	} else {
		besieged_count = 0;
	}

	bool focus_map_center = false;

	if (state == INIT) {
		go_back_home();
	} else if (state == STAY_HOME) {
		bool go_out =
		    try_goto_buff_zone() || (being_besieged && try_run_away());
		if (!go_out) {
			focus_map_center = true;
		}
		if (!go_out && !can_attack) {
			self_rotation.start();
		} else {
			if (self_rotation.stop() && !go_out) {
				go_back_home();
			}
		}
	} else if (state == GOING_OUT) {
	} else if (state == GOING_BACK) {
		try_goto_buff_zone() || (being_besieged && try_run_away());
	}

	publish_focus_center(focus_map_center);
}
