#include <iostream>
#include <roborts_msgs/GimbalAngle.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "pitch_calibration");

	double start = -.3;
	double end = .3;
	double step = .003;

	double ctrl = NAN;

	tf2_ros::Buffer tf_buf;
	tf2_ros::TransformListener tf_listener{tf_buf};
	ros::NodeHandle nh;
	ros::Publisher pub =
	    nh.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 1);

	ros::Rate loop(4);
	while (ros::ok()) {
		ros::spinOnce();
		loop.sleep();

		if (std::isnan(ctrl)) {
			ctrl = start;
		} else {
			geometry_msgs::TransformStamped transform;
			try {
				transform = tf_buf.lookupTransform(
				    "pitch_link", "pitch_base_link", ros::Time::now(),
				    ros::Duration(0.050));
			} catch (const std::exception &e) {
				ROS_WARN("%s", e.what());
				continue;
			}
			auto q = transform.transform.rotation;
			double roll, pitch, yaw;
			tf2::Matrix3x3{{q.x, q.y, q.z, q.w}}.getRPY(roll, pitch, yaw);
			std::cout << ctrl << "," << -pitch << std::endl;
			ctrl += step;
		}

		if (ctrl > end) {
			break;
		}

		roborts_msgs::GimbalAngle msg;
		msg.pitch_mode = false;
		msg.pitch_angle = ctrl;
		pub.publish(msg);
	}
}
