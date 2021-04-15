#include <roborts_msgs/RobotStatus.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_info_publisher");
	ros::NodeHandle nh;

	uint8_t robot_id = 0;
	auto robot_status_sub = nh.subscribe<roborts_msgs::RobotStatus>(
	    "robot_status", 1, [&](const roborts_msgs::RobotStatus::ConstPtr &msg) {
		    if (msg->id == robot_id) {
			    return;
		    }
		    robot_id = msg->id;
		    if (robot_id < 100) {
		    	ros::param::set("team_color", "red");
		    } else {
		    	ros::param::set("team_color", "blue");
		    }
	    });

	ros::spin();
}
