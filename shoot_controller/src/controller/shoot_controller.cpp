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
      heat_cooling_limit{0}, tf_buffer{ros::Duration{60}}, tf_listener{
                                                               tf_buffer} {
	ros::NodeHandle nh;
	gimbal_pub = nh.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 1);

	ros::param::get("~yaw_base_frame", yaw_base_frame);
	ros::param::get("~pitch_base_frame", pitch_base_frame);
	ros::param::get("~shoot_frame", shoot_frame);
	ros::param::get("~fixed_frame", fixed_frame);

	cmd_shoot = nh.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");

	game_status_sub = nh.subscribe<roborts_msgs::GameStatus>(
	    "/game_status", 10,
	    [this](const roborts_msgs::GameStatus::ConstPtr &msg) {
		    in_play = msg->game_status == roborts_msgs::GameStatus::GAME;
	    });

	robot_heat_sub = nh.subscribe<roborts_msgs::RobotHeat>(
	    "/robot_heat", 1, [this](const roborts_msgs::RobotHeat::ConstPtr &msg) {
		    heat = msg->shooter_heat;
	    });

	robot_status_sub = nh.subscribe<roborts_msgs::RobotStatus>(
	    "/robot_status", 1,
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
}

bool filter_result(const geometry_msgs::Point &pos) {
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
               const geometry_msgs::PointStamped &target_in_pitch_base,
               const geometry_msgs::PointStamped &target_in_yaw_base,
               const geometry_msgs::PointStamped &shoot_in_pitch_base,
               double bullet_velocity) {
	double gun_barrel_length =
	    std::sqrt(shoot_in_pitch_base.point.x * shoot_in_pitch_base.point.x +
	              shoot_in_pitch_base.point.y * shoot_in_pitch_base.point.y +
	              shoot_in_pitch_base.point.z * shoot_in_pitch_base.point.z);
	double distance =
	    std::sqrt(target_in_yaw_base.point.x * target_in_yaw_base.point.x +
	              target_in_yaw_base.point.y * target_in_yaw_base.point.y);
	double height = target_in_pitch_base.point.z;
	double pitch =
	    compute_pitch(distance, height, gun_barrel_length, bullet_velocity);

	if (std::isnan(pitch)) {
		ctrl.pitch_mode = true;
		ctrl.pitch_angle = 0;
	} else {
		double pitch_compensation = 0;
		ros::param::getCached("~pitch_compensation", pitch_compensation);
		ctrl.pitch_mode = false;
		ctrl.pitch_angle = -(pitch + pitch_compensation);
	}
}

void set_yaw(roborts_msgs::GimbalAngle &ctrl,
             const geometry_msgs::PointStamped &target_in_yaw_base) {
	ctrl.yaw_mode = false;
	ctrl.yaw_angle =
	    std::atan2(target_in_yaw_base.point.y, target_in_yaw_base.point.x);
}

bool ShootController::aim_and_shoot(
    const geometry_msgs::PointStamped &target_) {
	geometry_msgs::PointStamped target_in_pitch_base;
	geometry_msgs::PointStamped target_in_yaw_base;
	geometry_msgs::PointStamped shoot_in_pitch_base;
	shoot_in_pitch_base.header.frame_id = shoot_frame;
	shoot_in_pitch_base.header.stamp = target_.header.stamp;
	try {
		tf_buffer.transform(target_, target_in_pitch_base, pitch_base_frame,
		                    ros::Duration{0.1});
		tf_buffer.transform(target_, target_in_yaw_base, yaw_base_frame,
		                    ros::Duration{0.1});
		tf_buffer.transform(shoot_in_pitch_base, shoot_in_pitch_base,
		                    pitch_base_frame, ros::Duration{0.1});
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
	set_yaw(gimbal_ctrl, target_in_yaw_base);
	set_pitch(gimbal_ctrl, target_in_pitch_base, target_in_yaw_base,
	          shoot_in_pitch_base, speed_monitor.get_current_bullet_velocity());

	gimbal_pub.publish(gimbal_ctrl);

	if (in_play) {
		shoot();
	}

	return true;
}

bool ShootController::track(const geometry_msgs::PointStamped &target_) {
	geometry_msgs::PointStamped target_in_pitch_base;
	geometry_msgs::PointStamped target_in_yaw_base;
	geometry_msgs::PointStamped shoot_in_pitch_base;
	shoot_in_pitch_base.header.frame_id = shoot_frame;
	shoot_in_pitch_base.header.stamp = target_.header.stamp;
	try {
		tf_buffer.transform(target_, target_in_pitch_base, pitch_base_frame,
		                    ros::Time{0}, fixed_frame, ros::Duration{0.1});
		tf_buffer.transform(target_, target_in_yaw_base, yaw_base_frame,
		                    ros::Time{0}, fixed_frame, ros::Duration{0.1});
		tf_buffer.transform(shoot_in_pitch_base, shoot_in_pitch_base,
		                    pitch_base_frame, ros::Time{0}, pitch_base_frame,
		                    ros::Duration{0.1});
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
	set_yaw(gimbal_ctrl, target_in_yaw_base);
	set_pitch(gimbal_ctrl, target_in_pitch_base, target_in_yaw_base,
	          shoot_in_pitch_base, speed_monitor.get_current_bullet_velocity());

	gimbal_pub.publish(gimbal_ctrl);

	return true;
}

void ShootController::control_loop() {
	if (has_last_target) {
		track(last_target);
	}
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
