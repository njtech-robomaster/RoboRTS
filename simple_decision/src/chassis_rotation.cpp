#include "chassis_rotation.hpp"
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

double normalize_angle(double a) {
	while (a < -M_PI)
		a += 2 * M_PI;
	while (a > M_PI)
		a -= 2 * M_PI;
	return a;
}

constexpr double initial_speed = 0.3;

ChassisRotation::ChassisRotation() {
	ros::NodeHandle nh;
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	odom_sub = nh.subscribe<nav_msgs::Odometry>(
	    "odom", 1, [this](const nav_msgs::Odometry::ConstPtr &odom) {
		    auto q = odom->pose.pose.orientation;
		    double roll, pitch, yaw;
		    tf2::Matrix3x3{{q.x, q.y, q.z, q.w}}.getRPY(roll, pitch, yaw);

		    switch (state) {

		    case ACTIVE: {
			    double relative_angle = normalize_angle(yaw - initial_angle);
			    double t = (relative_angle - angle_min) / (angle_max - angle_min) * M_PI;

			    angle_speed_abs = std::sin(t) * (1 - initial_speed) + initial_speed;
			    if (relative_angle <= angle_min) {
				    angle_speed_sig = 1;
			    } else if (relative_angle >= angle_max) {
				    angle_speed_sig = -1;
			    }
		    }
		    break;

		    case INITIALIZING: {
			    initial_angle = yaw;
			    angle_speed_sig = 1;
			    angle_speed_abs = 1;
			    state = ACTIVE;
		    }
		    break;

		    case INACTIVE:
		    return;

		    }

		    geometry_msgs::Twist ctrl;
		    ctrl.angular.z = angle_speed * angle_speed_abs * angle_speed_sig;
		    cmd_vel_pub.publish(ctrl);
	    });
}

bool ChassisRotation::start() {
	if (state == INACTIVE) {
		state = INITIALIZING;
		return true;
	}
	return false;
}

bool ChassisRotation::stop() {
	if (state != INACTIVE) {
		geometry_msgs::Twist ctrl;
		cmd_vel_pub.publish(ctrl);
		state = INACTIVE;
		return true;
	}
	return false;
}
