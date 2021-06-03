#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace Goals {

geometry_msgs::Pose pose(double x, double y, double yaw) {
	geometry_msgs::Pose it;
	it.position.x = x;
	it.position.y = y;
	tf2::Quaternion q;
	q.setRPY(0, 0, yaw);
	it.orientation = tf2::toMsg(q);
	return it;
}

const double fix = .20;
const double fix2 = .40;
const double angle_fix = -M_PI / 4.0;
const double angle_fix2 = -M_PI / 6.0;

const geometry_msgs::Pose C1 = pose(-3.54, 0, 0);
const geometry_msgs::Pose C2 = pose(-3.54, -1.74 + fix, M_PI / 2 + angle_fix);
const geometry_msgs::Pose C3 = pose(3.54, 0, M_PI);
const geometry_msgs::Pose C4 = pose(3.54, 1.74 - fix, -M_PI / 2 + angle_fix);

geometry_msgs::Point point(double x, double y) {
	geometry_msgs::Point it;
	it.x = x;
	it.y = y;
	return it;
}

const geometry_msgs::Point F1 = point(-3.54, 0.55);
const geometry_msgs::Point F2 = point(-2.14, -0.59);
const geometry_msgs::Point F3 = point(0, 1.795);
const geometry_msgs::Point F4 = point(0, -1.795);
const geometry_msgs::Point F5 = point(2.14, 0.59);
const geometry_msgs::Point F6 = point(3.54, -0.55);

}; // namespace Goals
