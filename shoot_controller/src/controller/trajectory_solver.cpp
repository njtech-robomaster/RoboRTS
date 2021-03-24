#include "trajectory_solver.hpp"
#include <roborts_msgs/GimbalAngle.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

TrajectorySolver::TrajectorySolver() : tf_listener{tf_buffer} {
	ros::NodeHandle nh;
	gimbal_pub = nh.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 1);
}

double compute_pitch(double distance, double height, double gun_barrel_length,
                     double bullet_velocity) {
	double a = (9.8 * (distance - gun_barrel_length) *
	            (distance - gun_barrel_length)) /
	           (2 * bullet_velocity * bullet_velocity);
	double m = std::sqrt(distance * distance - 4 * a * (a + height));
	double t1 = std::atan2(distance + m, 2 * a);
	double t2 = std::atan2(distance - m, 2 * a);
	if (std::abs(t1) > std::abs(t2)) {
		return -t2;
	} else {
		return -t1;
	}
}

bool TrajectorySolver::aim_target(const geometry_msgs::PointStamped &target_) {
	geometry_msgs::PointStamped target_in_pitch_base;
	geometry_msgs::PointStamped target_in_yaw_base;
	geometry_msgs::PointStamped shoot_in_pitch_base;
	shoot_in_pitch_base.header.frame_id = shoot_frame;
	shoot_in_pitch_base.header.stamp = target_.header.stamp;
	try {
		tf_buffer.transform(target_, target_in_pitch_base, pitch_base_frame);
		tf_buffer.transform(target_, target_in_yaw_base, yaw_base_frame);
		tf_buffer.transform(shoot_in_pitch_base, shoot_in_pitch_base,
		                    pitch_base_frame);
	} catch (const std::exception &e) {
		ROS_WARN("Couldn't lookup transform: %s", e.what());
		return false;
	}
	double gun_barrel_length =
	    std::sqrt(shoot_in_pitch_base.point.x * shoot_in_pitch_base.point.x +
	              shoot_in_pitch_base.point.y * shoot_in_pitch_base.point.y +
	              shoot_in_pitch_base.point.z * shoot_in_pitch_base.point.z);

	roborts_msgs::GimbalAngle gimbal_ctrl;
	gimbal_ctrl.yaw_mode = false;
	gimbal_ctrl.pitch_mode = false;

	gimbal_ctrl.yaw_angle =
	    std::atan2(target_in_yaw_base.point.y, target_in_yaw_base.point.x);
	gimbal_ctrl.pitch_angle = compute_pitch(
	    std::sqrt(target_in_yaw_base.point.x * target_in_yaw_base.point.x +
	              target_in_yaw_base.point.y * target_in_yaw_base.point.y),
	    target_in_pitch_base.point.z, gun_barrel_length, bullet_velocity);

	gimbal_pub.publish(gimbal_ctrl);

	return true;
}
