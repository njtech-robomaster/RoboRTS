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
		    switch (robot_id) {
		    case 1:
			    ros::param::set("team_color", "red");
			    ros::param::set("robot_id", 1);
			    break;
		    case 2:
			    ros::param::set("team_color", "red");
			    ros::param::set("robot_id", 2);
			    break;
		    case 101:
			    ros::param::set("team_color", "blue");
			    ros::param::set("robot_id", 1);
			    break;
		    case 102:
			    ros::param::set("team_color", "blue");
			    ros::param::set("robot_id", 2);
			    break;
		    default:
			    ROS_WARN("Unrecognized robot id: %d", robot_id);
			    break;
		    }
	    });

	ros::spin();
}
