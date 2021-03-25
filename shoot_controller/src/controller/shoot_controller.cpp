#include "speed_monitor.hpp"
#include "trajectory_solver.hpp"
#include <geometry_msgs/PointStamped.h>
#include <roborts_msgs/RobotShoot.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "shoot_controller");

	TrajectorySolver trajectory_solver;
	ros::param::get("~yaw_base_frame", trajectory_solver.yaw_base_frame);
	ros::param::get("~pitch_base_frame", trajectory_solver.pitch_base_frame);
	ros::param::get("~shoot_frame", trajectory_solver.shoot_frame);

	ros::NodeHandle nh;

	auto armor_target_sub = nh.subscribe<geometry_msgs::PointStamped>(
	    "armor_target", 1,
	    [&](const geometry_msgs::PointStamped::ConstPtr &target) {
			ros::param::getCached("bullet_velocity", trajectory_solver.bullet_velocity);
		    trajectory_solver.aim_target(*target);
	    });

	SpeedMonitor speed_monitor;
	bool use_measured_velocity =
	    ros::param::param("~use_measured_velocity", true);
	auto shoot_sub = nh.subscribe<roborts_msgs::RobotShoot>(
	    "robot_shoot", 1, [&](const roborts_msgs::RobotShoot::ConstPtr &msg) {
		    double est_velocity = speed_monitor.update_velocity(msg->speed);
		    if (use_measured_velocity) {
			    ros::param::set("bullet_velocity", est_velocity);
			    ROS_INFO("Latest bullet velocity: %lf, estimated bullet "
			             "velocity: %lf",
			             msg->speed, est_velocity);
		    }
	    });

	ros::spin();
}
