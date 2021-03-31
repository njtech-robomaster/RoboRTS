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

ChassisRotation::ChassisRotation() {
	ros::NodeHandle nh;
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	odom_sub = nh.subscribe<nav_msgs::Odometry>(
	    "odom", 1, [this](const nav_msgs::Odometry::ConstPtr &odom) {
		    auto q = odom->pose.pose.orientation;
		    double roll, pitch, yaw;
		    tf2::Matrix3x3{{q.x, q.y, q.z, q.w}}.getRPY(roll, pitch, yaw);
		    double relative_angle = normalize_angle(yaw - initial_angle);

		    switch (state) {
		    case ACTIVE:
			    if (relative_angle <= angle_min) {
				    angle_speed_ctrl = angle_speed;
			    } else if (relative_angle >= angle_max) {
				    angle_speed_ctrl = -angle_speed;
			    }
			    break;
		    case INITIALIZING:
			    initial_angle = yaw;
			    angle_speed_ctrl = angle_speed;
			    state = ACTIVE;
			    break;
		    case INACTIVE:
			    return;
		    }

		    geometry_msgs::Twist ctrl;
		    ctrl.angular.z = angle_speed_ctrl;
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
