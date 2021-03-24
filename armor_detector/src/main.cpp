#include "detector.hpp"
#include <chrono>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "armor_detector");

	RSArmorDetector detector{
	    {ros::param::param("~color_resolution_width", 1920),
	     ros::param::param("~color_resolution_height", 1080)},
	    {ros::param::param("~depth_resolution_width", 1280),
	     ros::param::param("~depth_resolution_height", 720)},
	};

	ros::NodeHandle nh;
	ros::Publisher result_pub =
	    nh.advertise<geometry_msgs::PointStamped>("armor_target", 1);
	geometry_msgs::PointStamped msg;
	msg.header.frame_id =
	    ros::param::param<std::string>("~camera_frame", "camera_link");

	auto last_poll_time = std::chrono::high_resolution_clock::now();
	ros::Rate loop_rate(60);
	while (ros::ok()) {

		msg.header.stamp = ros::Time::now();
		if (detector.poll()) {
			DetectResult result;
			if (detector.detect(result)) {
				msg.point.x = result.position.x;
				msg.point.y = result.position.y;
				msg.point.z = result.position.z;
				result_pub.publish(msg);
			}
			last_poll_time = std::chrono::high_resolution_clock::now();
		}

		int ms_elapsed =
		    std::chrono::duration_cast<std::chrono::milliseconds>(
		        std::chrono::high_resolution_clock::now() - last_poll_time)
		        .count();
		if (ms_elapsed > 1000) {
			ROS_WARN("No frame data is received for %d ms", ms_elapsed);
			if (ms_elapsed > 5000) {
				ROS_ERROR("No data received in 5s, exiting...");
				abort();
			}
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}
