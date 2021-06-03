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

const geometry_msgs::Pose C1 = pose(-3.54 + fix2, 1.74, 0 + angle_fix2);
const geometry_msgs::Pose C2 = pose(-3.54, -1.74 + fix, M_PI / 2 + angle_fix);
const geometry_msgs::Pose C3 = pose(3.54 - fix2, -1.74, M_PI + angle_fix2);
const geometry_msgs::Pose C4 = pose(3.54, 1.74 - fix, -M_PI / 2 + angle_fix);

const double buff_zone_orientation = M_PI;

const geometry_msgs::Pose F1 = pose(-3.54, 0.55, 0);
const geometry_msgs::Pose F2 = pose(-2.14, -0.59, buff_zone_orientation);
const geometry_msgs::Pose F3 = pose(0, 1.795, buff_zone_orientation);
const geometry_msgs::Pose F4 = pose(0, -1.795, buff_zone_orientation);
const geometry_msgs::Pose F5 = pose(2.14, 0.59, buff_zone_orientation);
const geometry_msgs::Pose F6 = pose(3.54, -0.55, M_PI);

}; // namespace Goals
