#include "trajectory_solver.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "shoot_controller");

	TrajectorySolver trajectory_solver;
	ros::param::get("base_frame", trajectory_solver.base_frame);
	ros::param::get("shoot_frame", trajectory_solver.shoot_frame);
	ros::param::get("sim_time_step", trajectory_solver.sim_time_step);

	ros::Rate loop(10);
	while (ros::ok()) {

		ros::param::getCached("bullet_velocity",
		                      trajectory_solver.bullet_velocity);

		trajectory_solver.publish_simulation();

		ros::spinOnce();
		loop.sleep();
	}
}
