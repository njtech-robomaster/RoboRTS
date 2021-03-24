#include "trajectory.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "shoot_controller");

	TrajectoryPublisher trajectory_pub;

	ros::Rate loop(10);
	while (ros::ok()) {

		trajectory_pub.publish();

		ros::spinOnce();
		loop.sleep();
	}
}
