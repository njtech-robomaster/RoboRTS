#include "speed_monitor.hpp"
#include "trajectory_solver.hpp"
#include <geometry_msgs/PointStamped.h>
#include <roborts_msgs/RobotShoot.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "shoot_controller");
	ros::NodeHandle nh;

	TrajectorySolver trajectory_solver;
	ros::param::get("~yaw_base_frame", trajectory_solver.yaw_base_frame);
	ros::param::get("~pitch_base_frame", trajectory_solver.pitch_base_frame);
	ros::param::get("~shoot_frame", trajectory_solver.shoot_frame);
	ros::param::get("initial_bullet_velocity",
	                trajectory_solver.bullet_velocity);
	auto armor_target_sub = nh.subscribe<geometry_msgs::PointStamped>(
	    "armor_target", 1,
	    [&](const geometry_msgs::PointStamped::ConstPtr &target) {
		    trajectory_solver.aim_target(*target);
	    });

	bool use_measured_velocity =
	    ros::param::param("~use_measured_velocity", true);
	auto current_bullet_velocity_pub =
	    nh.advertise<std_msgs::Float64>("current_bullet_velocity", 1, true);
	SpeedMonitor speed_monitor;
	auto shoot_sub = nh.subscribe<roborts_msgs::RobotShoot>(
	    "robot_shoot", 1, [&](const roborts_msgs::RobotShoot::ConstPtr &msg) {
		    double est_velocity = speed_monitor.update_velocity(msg->speed);
		    if (use_measured_velocity) {
			    trajectory_solver.bullet_velocity = est_velocity;
			    std_msgs::Float64 vec_msg;
			    vec_msg.data = est_velocity;
			    current_bullet_velocity_pub.publish(vec_msg);
			    ROS_INFO("Latest bullet velocity: %lf, estimated bullet "
			             "velocity: %lf",
			             msg->speed, est_velocity);
		    }
	    });

	ros::spin();
}
