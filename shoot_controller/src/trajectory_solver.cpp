#include "trajectory_solver.hpp"
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const double g = 9.8;

nav_msgs::Path simulate_trajectory(std::string tf, geometry_msgs::Point x0,
                                   geometry_msgs::Vector3 v0,
                                   double time_step_) {
	nav_msgs::Path path;
	path.header.frame_id = tf;
	ros::Time t0 = ros::Time::now();
	path.header.stamp = t0;
	ros::Duration dt{time_step_};

	for (int i = 0;; i++) {
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = tf;
		pose.header.stamp = t0 + dt * i;

		double t = time_step_ * i;
		pose.pose.position.x = x0.x + v0.x * t;
		pose.pose.position.y = x0.y + v0.y * t;
		pose.pose.position.z = x0.z + v0.z * t - g * t * t / 2.0;

		if (pose.pose.position.z < 0) {
			break;
		}

		path.poses.push_back(pose);
	}
	return path;
}

TrajectorySolver::TrajectorySolver() : tf_listener{tf_buffer} {
	ros::NodeHandle nh;
	trajectory_path_pub = nh.advertise<nav_msgs::Path>("bullet_trajectory", 1);
}

bool TrajectorySolver::publish_simulation() {
	geometry_msgs::TransformStamped transform;

	try {
		transform = tf_buffer.lookupTransform(
		    base_frame, shoot_frame, ros::Time::now(), ros::Duration{1.0});
	} catch (const std::exception &e) {
		ROS_WARN("Couldn't lookup transform: %s", e.what());
		return false;
	}

	geometry_msgs::Vector3 v0;
	v0.x = bullet_velocity;
	v0.y = 0;
	v0.z = 0;
	tf2::doTransform(v0, v0, transform);

	geometry_msgs::Point x0;
	x0.x = 0;
	x0.y = 0;
	x0.z = 0;
	tf2::doTransform(x0, x0, transform);

	nav_msgs::Path path =
	    simulate_trajectory(base_frame, x0, v0, sim_time_step);
	trajectory_path_pub.publish(path);
	return true;
}
