#include "speed_monitor.hpp"
#include <roborts_msgs/RobotShoot.h>
#include <std_msgs/Float64.h>

SpeedMonitor::SpeedMonitor() {
	ros::NodeHandle nh;

	ros::param::get("~use_measured_velocity", use_measured_velocity);
	ros::param::get("initial_bullet_velocity", current_bullet_velocity);

	current_bullet_velocity_pub =
	    nh.advertise<std_msgs::Float64>("current_bullet_velocity", 1, true);

	robot_shoot_sub = nh.subscribe<roborts_msgs::RobotShoot>(
	    "robot_shoot", 1,
	    [this](const roborts_msgs::RobotShoot::ConstPtr &msg) {
		    double est_velocity = update_velocity(msg->speed);
		    if (use_measured_velocity) {
			    current_bullet_velocity = est_velocity;
			    std_msgs::Float64 vec_msg;
			    vec_msg.data = est_velocity;
			    current_bullet_velocity_pub.publish(vec_msg);
			    ROS_INFO("Latest bullet velocity: %lf, estimated bullet "
			             "velocity: %lf",
			             msg->speed, est_velocity);
		    }
	    });
}

double SpeedMonitor::update_velocity(double latest_velocity) {
	// TODO drop invalid velocity
	return latest_velocity;
}

double SpeedMonitor::get_current_bullet_velocity() const {
	return current_bullet_velocity;
}
