#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

class TrajectoryVisualization {
  public:
	TrajectoryVisualization() : tf_listener{tf_buffer} {
		ros::NodeHandle nh;
		trajectory_path_pub =
		    nh.advertise<nav_msgs::Path>("bullet_trajectory", 1);
		ros::param::get("~base_frame", base_frame);
		ros::param::get("~shoot_frame", shoot_frame);
		ros::param::get("~sim_time_step", sim_time_step);
	}

	bool publish_simulation() {
		geometry_msgs::TransformStamped transform;

		try {
			transform = tf_buffer.lookupTransform(
			    base_frame, shoot_frame, ros::Time::now(), ros::Duration{0.05});
		} catch (const std::exception &e) {
			ROS_WARN("Couldn't lookup transform: %s", e.what());
			return false;
		}

		double bullet_velocity = 8.0;
		ros::param::getCached("bullet_velocity", bullet_velocity);

		geometry_msgs::Vector3 v0;
		v0.x = bullet_velocity;
		v0.y = 0;
		v0.z = 0;
		tf2::doTransform(v0, v0, transform);

		geometry_msgs::Point x0;
		x0.x = 0;
		x0.y = 0;
		x0.z = 0;
		tf2::doTransform(x0, x0, transform);

		trajectory_path_pub.publish(simulate_trajectory(x0, v0));
		return true;
	}

  private:
	ros::Publisher trajectory_path_pub;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;
	double sim_time_step = 0.01;
	std::string base_frame = "base_link";
	std::string shoot_frame = "shoot_link";

	nav_msgs::Path simulate_trajectory(geometry_msgs::Point x0,
	                                   geometry_msgs::Vector3 v0) {
		nav_msgs::Path path;
		path.header.frame_id = base_frame;
		ros::Time t0 = ros::Time::now();
		path.header.stamp = t0;
		ros::Duration dt{sim_time_step};

		for (int i = 0;; i++) {
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = base_frame;
			pose.header.stamp = t0 + dt * i;

			double t = sim_time_step * i;
			pose.pose.position.x = x0.x + v0.x * t;
			pose.pose.position.y = x0.y + v0.y * t;
			pose.pose.position.z = x0.z + v0.z * t - 9.8 * t * t / 2.0;

			if (pose.pose.position.z < 0) {
				break;
			}

			path.poses.push_back(pose);
		}
		return path;
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "trajectory_visualization");

	TrajectoryVisualization node;

	ros::Rate loop(20);
	while (ros::ok()) {
		node.publish_simulation();
		ros::spinOnce();
		loop.sleep();
	}
}
