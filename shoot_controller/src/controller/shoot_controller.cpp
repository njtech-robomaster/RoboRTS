#include "speed_monitor.hpp"
#include "trajectory_solver.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "shoot_controller");
	ros::NodeHandle nh;

	SpeedMonitor speed_monitor;
	TrajectorySolver trajectory_solver{
	    std::bind(&SpeedMonitor::get_current_bullet_velocity, speed_monitor)};

	ros::spin();
}
