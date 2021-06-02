#include "shoot_controller.hpp"
#include "trajectory_solver.hpp"
#include <roborts_msgs/GameStatus.h>
#include <roborts_msgs/GimbalAngle.h>
#include <roborts_msgs/RobotHeat.h>
#include <roborts_msgs/RobotStatus.h>
#include <roborts_msgs/ShootCmd.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ShootController::ShootController()
    : has_last_target{false}, in_play{false}, heat{0}, heat_cooling_rate{0},
      heat_cooling_limit{0}, has_moving_reference{false},
      tf_buffer{ros::Duration{5.0}}, tf_listener{tf_buffer} {
	ros::NodeHandle nh;
	gimbal_pub = nh.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 1);

	ros::param::get("~yaw_base_frame", yaw_base_frame);
	ros::param::get("~pitch_base_frame", pitch_base_frame);
	ros::param::get("~fixed_frame", fixed_frame);
	ros::param::get("~gun_barrel_length", gun_barrel_length);
	ROS_INFO("Gun barrel length is %lf", gun_barrel_length);
	ros::param::get("~lookout_frame", lookout_frame);

	cmd_shoot = nh.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");

	game_status_sub = nh.subscribe<roborts_msgs::GameStatus>(
	    "game_status", 10,
	    [this](const roborts_msgs::GameStatus::ConstPtr &msg) {
		    in_play = msg->game_status == roborts_msgs::GameStatus::GAME;
	    });

	robot_heat_sub = nh.subscribe<roborts_msgs::RobotHeat>(
	    "robot_heat", 1, [this](const roborts_msgs::RobotHeat::ConstPtr &msg) {
		    heat = msg->shooter_heat;
	    });

	robot_status_sub = nh.subscribe<roborts_msgs::RobotStatus>(
	    "robot_status", 1,
	    [this](const roborts_msgs::RobotStatus::ConstPtr &msg) {
		    heat_cooling_rate = msg->heat_cooling_rate;
		    heat_cooling_limit = msg->heat_cooling_limit;
	    });

	armor_target_sub = nh.subscribe<geometry_msgs::PointStamped>(
	    "armor_target", 1,
	    [this](const geometry_msgs::PointStamped::ConstPtr &target) {
		    aim_and_shoot(*target);
	    });

	control_loop_timer =
	    nh.createTimer(ros::Duration{0.01},
	                   [this](const ros::TimerEvent &) { control_loop(); });

	yaw_focus_sub = nh.subscribe<roborts_msgs::YawFocus>(
	    "cmd_yaw_focus", 1,
	    [this](const roborts_msgs::YawFocus::ConstPtr &msg) {
		    yaw_focus = *msg;
	    });
}

bool filter_result(const geometry_msgs::Point &pos) {
	if (pos.x < -1.0) {
		return false;
	}

	double height_min = 0.0;
	ros::param::getCached("~target_height_min", height_min);
	if (pos.z < height_min)
		return false;

	double height_max = 0.5;
	ros::param::getCached("~target_height_max", height_max);
	if (pos.z > height_max)
		return false;

	double ground_distance = std::sqrt(pos.x * pos.x + pos.y * pos.y);

	double distance_min = 0.0;
	ros::param::getCached("~target_ground_distance_min", distance_min);
	if (ground_distance < distance_min)
		return false;

	double distance_max = 10.0;
	ros::param::getCached("~target_ground_distance_max", distance_max);
	if (ground_distance > distance_max)
		return false;

	return true;
}

void set_pitch(roborts_msgs::GimbalAngle &ctrl,
               const geometry_msgs::Point &target_in_pitch_base,
               const geometry_msgs::Point &target_in_yaw_base,
               double bullet_velocity, double gun_barrel_length) {
	double distance = std::sqrt(target_in_yaw_base.x * target_in_yaw_base.x +
	                            target_in_yaw_base.y * target_in_yaw_base.y);
	double height = target_in_pitch_base.z;
	double pitch =
	    compute_pitch(distance, height, gun_barrel_length, bullet_velocity);

	ctrl.pitch_mode = false;
	if (std::isnan(pitch)) {
		ctrl.pitch_angle = 0;
	} else {
		double pitch_compensation = 0;
		ros::param::getCached("~pitch_compensation", pitch_compensation);
		ctrl.pitch_angle = -(pitch + pitch_compensation);
	}
}

void set_yaw(roborts_msgs::GimbalAngle &ctrl,
             const geometry_msgs::Point &target_in_yaw_base) {
	ctrl.yaw_mode = false;
	ctrl.yaw_angle = std::atan2(target_in_yaw_base.y, target_in_yaw_base.x);
}

bool ShootController::aim_and_shoot(
    const geometry_msgs::PointStamped &target_) {
	geometry_msgs::PointStamped target_in_pitch_base;
	geometry_msgs::PointStamped target_in_yaw_base;
	try {
		tf_buffer.transform(target_, target_in_pitch_base, pitch_base_frame,
		                    ros::Duration{0.1});
		tf_buffer.transform(target_, target_in_yaw_base, yaw_base_frame,
		                    ros::Duration{0.1});
	} catch (const std::exception &e) {
		ROS_WARN("Couldn't lookup transform: %s", e.what());
		return false;
	}

	if (!filter_result(target_in_yaw_base.point)) {
		ROS_INFO("Invalid detection result: %f, %f, %f",
		         target_in_yaw_base.point.x, target_in_yaw_base.point.y,
		         target_in_yaw_base.point.z);
		return false;
	}

	has_last_target = true;
	last_target = target_;

	bool debug_print_target = false;
	ros::param::getCached("~debug_print_target", debug_print_target);
	if (debug_print_target) {
		ROS_INFO("Target: %f, %f, %f", target_in_yaw_base.point.x,
		         target_in_yaw_base.point.y, target_in_yaw_base.point.z);
	}

	roborts_msgs::GimbalAngle gimbal_ctrl;
	set_yaw(gimbal_ctrl, target_in_yaw_base.point);
	set_pitch(gimbal_ctrl, target_in_pitch_base.point, target_in_yaw_base.point,
	          speed_monitor.get_current_bullet_velocity(), gun_barrel_length);

	gimbal_pub.publish(gimbal_ctrl);

	if (in_play) {
		shoot();
	}

	return true;
}

bool ShootController::track(const geometry_msgs::PointStamped &target_) {
	geometry_msgs::PointStamped target_in_pitch_base;
	geometry_msgs::PointStamped target_in_yaw_base;
	try {
		tf_buffer.transform(target_, target_in_pitch_base, pitch_base_frame,
		                    ros::Time{0}, fixed_frame, ros::Duration{0.1});
		tf_buffer.transform(target_, target_in_yaw_base, yaw_base_frame,
		                    ros::Time{0}, fixed_frame, ros::Duration{0.1});
	} catch (const tf2::ExtrapolationException &e) {
		if (target_.header.stamp + tf_buffer.getCacheLength() <
		    ros::Time::now()) {
			has_last_target = false;
		} else {
			ROS_WARN("Couldn't lookup transform (tracking): %s", e.what());
		}
		return false;
	} catch (const std::exception &e) {
		ROS_WARN("Couldn't lookup transform (tracking): %s", e.what());
		return false;
	}

	if (!filter_result(target_in_yaw_base.point)) {
		ROS_INFO("Invalid tracking target: %f, %f, %f",
		         target_in_yaw_base.point.x, target_in_yaw_base.point.y,
		         target_in_yaw_base.point.z);
		return false;
	}

	roborts_msgs::GimbalAngle gimbal_ctrl;
	set_yaw(gimbal_ctrl, target_in_yaw_base.point);
	set_pitch(gimbal_ctrl, target_in_pitch_base.point, target_in_yaw_base.point,
	          speed_monitor.get_current_bullet_velocity(), gun_barrel_length);

	gimbal_pub.publish(gimbal_ctrl);

	return true;
}

void ShootController::control_loop() {
	bool use_moving_reference = false;
	if (has_last_target) {
		track(last_target);
	} else if (track_enemy_cars()) {
	} else if (yaw_focus.has_focus) {
		track_yaw_focus(yaw_focus.focus_point);
	} else {
		use_moving_reference = true;
	}
	track_moving_reference(use_moving_reference);
}

bool ShootController::shoot() {
	int bullets_per_shoot = 1;
	ros::param::getCached("~bullets_per_shoot", bullets_per_shoot);

	if (heat >
	    heat_cooling_limit - 3 * bullets_per_shoot *
	                             speed_monitor.get_current_bullet_velocity()) {
		// reserved heat = 3 * estimated heat per shoot
		// one for the bullet to be emitted
		// one for the pending bullet (if any)
		// one for the safety margin
		return false;
	}

	roborts_msgs::ShootCmd srv;
	srv.request.mode = roborts_msgs::ShootCmd::Request::ONCE;
	srv.request.number = bullets_per_shoot;
	cmd_shoot.call(srv);
	return true;
}

void ShootController::switch_moving_reference(bool control_gimbal) {
	moving_reference.header.frame_id = yaw_base_frame;
	moving_reference.header.stamp = ros::Time::now();
	moving_reference.point.x = 30.0;
	ros::param::getCached("~moving_reference_distance",
	                      moving_reference.point.x);
	has_moving_reference = true;

	if (control_gimbal) {
		roborts_msgs::GimbalAngle gimbal_ctrl;
		gimbal_ctrl.pitch_mode = false;
		gimbal_ctrl.pitch_angle = 0;
		gimbal_ctrl.yaw_mode = false;
		gimbal_ctrl.yaw_angle = 0;
		gimbal_pub.publish(gimbal_ctrl);
	}
}

void ShootController::track_moving_reference(bool control_gimbal) {
	if (!has_moving_reference) {
		switch_moving_reference(control_gimbal);
		return;
	}

	auto moving_reference_duration = tf_buffer.getCacheLength();
	if (moving_reference.header.stamp + moving_reference_duration <
	    ros::Time::now()) {
		switch_moving_reference(control_gimbal);
		return;
	}

	geometry_msgs::PointStamped reference_in_yaw_base;
	try {
		tf_buffer.transform(moving_reference, reference_in_yaw_base,
		                    yaw_base_frame, ros::Time{0}, fixed_frame,
		                    ros::Duration{0.1});
	} catch (const std::exception &e) {
		ROS_WARN("Couldn't lookup transform (moving reference): %s", e.what());
		return;
	}

	roborts_msgs::GimbalAngle gimbal_ctrl;
	set_yaw(gimbal_ctrl, reference_in_yaw_base.point);

	double angle_max = 1.0;
	ros::param::getCached("~moving_reference_max_angle", angle_max);
	if (std::abs(gimbal_ctrl.yaw_angle) > angle_max) {
		switch_moving_reference(control_gimbal);
		return;
	}

	if (control_gimbal) {
		gimbal_ctrl.pitch_mode = false;
		gimbal_ctrl.pitch_angle = 0;
		gimbal_pub.publish(gimbal_ctrl);
	}
}

bool ShootController::track_yaw_focus(
    const geometry_msgs::PointStamped &target) {
	geometry_msgs::PointStamped target_in_yaw_base;
	try {
		tf_buffer.transform(target, target_in_yaw_base, yaw_base_frame,
		                    ros::Time{0}, fixed_frame, ros::Duration{0.1});
	} catch (const std::exception &e) {
		ROS_WARN("Couldn't lookup transform (moving reference): %s", e.what());
		return false;
	}

	roborts_msgs::GimbalAngle gimbal_ctrl;
	set_yaw(gimbal_ctrl, target_in_yaw_base.point);
	gimbal_ctrl.pitch_mode = false;
	gimbal_ctrl.pitch_angle = 0;
	gimbal_pub.publish(gimbal_ctrl);
	return true;
}

double norm_squared(const geometry_msgs::Point &p) {
	return p.x * p.x + p.y * p.y + p.z * p.z;
}

double norm(const geometry_msgs::Point &p) {
	return std::sqrt(norm_squared(p));
}

bool ShootController::track_enemy_cars() {
	std::vector<geometry_msgs::Point> enemy_cars;
	std::vector<geometry_msgs::Point> unknown_cars;

	VehicleColor my_color;
	std::string team_color_str;
	ros::param::getCached("team_color", team_color_str);
	if (team_color_str == "red") {
		my_color = VehicleColor::RED;
	} else if (team_color_str == "blue") {
		my_color = VehicleColor::BLUE;
	} else {
		ROS_WARN("unrecognized team_color param: %s", team_color_str.c_str());
		return false;
	}

	geometry_msgs::TransformStamped transform;
	try {
		transform = tf_buffer.lookupTransform(yaw_base_frame, lookout_frame,
		                                      ros::Time{0});
	} catch (const std::exception &e) {
		ROS_WARN("Couldn't lookup transform (lookout tracking): %s", e.what());
		return false;
	}

	for (auto &car : lookout.get_cars()) {
		if (car.color == my_color) {
			continue;
		}

		geometry_msgs::Point position;
		position.x = car.x;
		position.y = car.y;
		geometry_msgs::Point relative_position;
		tf2::doTransform(position, relative_position, transform);
		if (relative_position.x < -1.0 || norm(relative_position) < 0.3) {
			continue;
		}

		if (car.color == VehicleColor::UNKNOWN) {
			unknown_cars.push_back(relative_position);
		} else {
			enemy_cars.push_back(relative_position);
		}
	}

	auto &cars_to_track = enemy_cars;
	if (cars_to_track.empty()) {
		return false;
	}

	std::sort(cars_to_track.begin(), cars_to_track.end(),
	          [](geometry_msgs::Point &a, geometry_msgs::Point &b) {
		          return norm_squared(a) < norm_squared(b);
	          });
	auto target = cars_to_track[0];

	ROS_INFO("Tracking lookout target at (relative) %lf, %lf", target.x,
	         target.y);

	roborts_msgs::GimbalAngle gimbal_ctrl;
	set_yaw(gimbal_ctrl, target);
	gimbal_ctrl.pitch_mode = false;
	gimbal_ctrl.pitch_angle = 0;
	gimbal_pub.publish(gimbal_ctrl);
	return true;
}
