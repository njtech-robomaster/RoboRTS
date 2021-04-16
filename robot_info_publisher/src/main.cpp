#include <roborts_msgs/CurrentLimit.h>
#include <roborts_msgs/RobotHeat.h>
#include <roborts_msgs/RobotStatus.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_info_publisher");
	ros::NodeHandle nh;

	auto current_limit_pub = nh.advertise<roborts_msgs::CurrentLimit>(
	    "cmd_chassis_current_limit", 1, true);

	uint8_t robot_id = 0;
	double power_limit = -1.;

	auto robot_status_sub = nh.subscribe<roborts_msgs::RobotStatus>(
	    "robot_status", 1, [&](const roborts_msgs::RobotStatus::ConstPtr &msg) {
		    if (msg->id != robot_id) {
			    robot_id = msg->id;
			    if (robot_id < 100) {
				    ros::param::set("team_color", "red");
			    } else {
				    ros::param::set("team_color", "blue");
			    }
		    }

		    power_limit = msg->chassis_power_limit;
	    });

	auto robot_heat_sub = nh.subscribe<roborts_msgs::RobotHeat>(
	    "robot_heat", 1, [&](const roborts_msgs::RobotHeat::ConstPtr &msg) {
		    roborts_msgs::CurrentLimit limit_ctrl;
		    if (power_limit > 0) {
			    double chassis_volt = msg->chassis_volt / 1000.0;
			    double max_current = power_limit / chassis_volt;
			    limit_ctrl.is_limited = true;
			    limit_ctrl.current_limit = max_current;
		    } else {
			    limit_ctrl.is_limited = false;
		    }
		    current_limit_pub.publish(limit_ctrl);
	    });

	ros::spin();
}
