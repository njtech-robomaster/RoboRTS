#include "shoot_controller.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "shoot_controller");
	ShootController shoot_controller;
	ros::spin();
}
