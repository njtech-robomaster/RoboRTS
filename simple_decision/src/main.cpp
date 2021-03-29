#include "decision_node.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "simple_decision");
	DecisionNode node;
	ros::spin();
}
