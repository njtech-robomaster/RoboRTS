#include "trajectory_solver.hpp"
#include <roborts_msgs/GimbalAngle.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

TrajectorySolver::TrajectorySolver(std::function<double()> bullet_velocity_fn_)
    : has_last_target{false}, bullet_velocity_fn{bullet_velocity_fn_},
      tf_buffer{ros::Duration{60}}, tf_listener{tf_buffer} {
	ros::NodeHandle nh;
	gimbal_pub = nh.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 1);

	ros::param::get("~yaw_base_frame", yaw_base_frame);
	ros::param::get("~pitch_base_frame", pitch_base_frame);
	ros::param::get("~shoot_frame", shoot_frame);

	armor_target_sub = nh.subscribe<geometry_msgs::PointStamped>(
	    "armor_target", 1,
	    [this](const geometry_msgs::PointStamped::ConstPtr &target) {
		    if (aim_target(*target)) {
			    has_last_target = true;
			    last_target = *target;
		    }
	    });

	control_loop_timer =
	    nh.createTimer(ros::Duration{0.01},
	                   [this](const ros::TimerEvent &) { control_loop(); });
}

double calc_polynomial(const std::vector<double> &coeffs, double x) {
	double y = 0;
	for (size_t k = 0; k < coeffs.size(); k++) {
		y += coeffs[k] * std::pow(x, k);
	}
	return y;
}

double calc_polynomial_derivative(const std::vector<double> &coeffs, double x) {
	double y = 0;
	for (size_t k = 1; k < coeffs.size(); k++) {
		y += k * coeffs[k] * std::pow(x, k - 1);
	}
	return y;
}

double newton_iteration(const std::vector<double> &coeffs, double x0,
                        double *err_ptr = nullptr, double max_err = 1e-6,
                        int max_iterations = 100) {
	int iterations = 0;
	double y;
	while (y = calc_polynomial(coeffs, x0), std::abs(y) > max_err) {
		if (iterations++ > max_iterations) {
			break;
		}
		x0 = x0 - y / calc_polynomial_derivative(coeffs, x0);
	}
	if (err_ptr != nullptr) {
		*err_ptr = y;
	}
	return x0;
}

const double g = 9.8;

double compute_pitch(double distance, double height, double gun_barrel_length,
                     double bullet_velocity) {
	std::vector<double> coeffs = {
	    -2 * g * gun_barrel_length * distance +
	        2 * height * bullet_velocity * bullet_velocity +
	        g * distance * distance + g * gun_barrel_length * gun_barrel_length,
	    -4 * bullet_velocity * bullet_velocity * distance,
	    -2 * g * gun_barrel_length * distance + 3 * g * distance * distance -
	        2 * height * bullet_velocity * bullet_velocity -
	        g * gun_barrel_length * gun_barrel_length,
	    +8 * bullet_velocity * bullet_velocity * distance,
	    +2 * g * gun_barrel_length * distance + 3 * g * distance * distance -
	        2 * height * bullet_velocity * bullet_velocity -
	        g * gun_barrel_length * gun_barrel_length,
	    -4 * bullet_velocity * bullet_velocity * distance,
	    +2 * g * gun_barrel_length * distance +
	        2 * height * bullet_velocity * bullet_velocity +
	        g * distance * distance +
	        g * gun_barrel_length * gun_barrel_length};
	double err;
	double k = newton_iteration(coeffs, 0, &err);
	if (err > 1e-3) {
		return NAN;
	}
	return 2 * std::atan(k);
}

bool filter_result(const geometry_msgs::Point &pos) {
	double height_min = 0.0;
	ros::param::getCached("~target_height_min", height_min);
	if (pos.z < height_min)
		return false;

	double height_max = 0.5;
	ros::param::getCached("~target_height_max", height_max);
	if (pos.z > height_max)
		return false;

	double ground_distance = std::sqrt(pos.x * pos.x + pos.y * pos.y);

	double distance_min = 0.0;
	ros::param::getCached("~target_ground_distance_min", distance_min);
	if (ground_distance < distance_min)
		return false;

	double distance_max = 10.0;
	ros::param::getCached("~target_ground_distance_max", distance_max);
	if (ground_distance > distance_max)
		return false;

	return true;
}

bool TrajectorySolver::aim_target(const geometry_msgs::PointStamped &target_) {
	std::string fixed_frame = "odom";
	ros::param::getCached("~fixed_frame", fixed_frame);

	geometry_msgs::PointStamped target_in_pitch_base;
	geometry_msgs::PointStamped target_in_yaw_base;
	geometry_msgs::PointStamped shoot_in_pitch_base;
	shoot_in_pitch_base.header.frame_id = shoot_frame;
	shoot_in_pitch_base.header.stamp = target_.header.stamp;
	try {
		tf_buffer.transform(target_, target_in_pitch_base, pitch_base_frame);
		tf_buffer.transform(target_, target_in_yaw_base, yaw_base_frame,
		                    ros::Time{0}, fixed_frame);
		tf_buffer.transform(shoot_in_pitch_base, shoot_in_pitch_base,
		                    pitch_base_frame, ros::Time{0}, fixed_frame);
	} catch (const std::exception &e) {
		ROS_WARN("Couldn't lookup transform: %s", e.what());
		return false;
	}

	if (!filter_result(target_in_yaw_base.point)) {
		ROS_INFO("Invalid detection result: %f, %f, %f",
		         target_in_yaw_base.point.x, target_in_yaw_base.point.y,
		         target_in_yaw_base.point.z);
		return false;
	}

	bool debug_print_target = false;
	ros::param::getCached("~debug_print_target", debug_print_target);
	if (debug_print_target) {
		ROS_INFO("Target: %f, %f, %f",
		         target_in_yaw_base.point.x, target_in_yaw_base.point.y,
		         target_in_yaw_base.point.z);
	}

	double gun_barrel_length =
	    std::sqrt(shoot_in_pitch_base.point.x * shoot_in_pitch_base.point.x +
	              shoot_in_pitch_base.point.y * shoot_in_pitch_base.point.y +
	              shoot_in_pitch_base.point.z * shoot_in_pitch_base.point.z);
	double distance =
	    std::sqrt(target_in_yaw_base.point.x * target_in_yaw_base.point.x +
	              target_in_yaw_base.point.y * target_in_yaw_base.point.y);
	double height = target_in_pitch_base.point.z;
	double pitch = -compute_pitch(distance, height, gun_barrel_length,
	                              bullet_velocity_fn());

	roborts_msgs::GimbalAngle gimbal_ctrl;
	gimbal_ctrl.yaw_mode = false;
	gimbal_ctrl.yaw_angle =
	    std::atan2(target_in_yaw_base.point.y, target_in_yaw_base.point.x);

	if (std::isnan(pitch)) {
		gimbal_ctrl.pitch_mode = true;
		gimbal_ctrl.pitch_angle = 0;
	} else {
		double pitch_compensation = 0;
		ros::param::getCached("~pitch_compensation", pitch_compensation);
		gimbal_ctrl.pitch_mode = false;
		gimbal_ctrl.pitch_angle = pitch - pitch_compensation;
	}

	gimbal_pub.publish(gimbal_ctrl);

	return true;
}

void TrajectorySolver::control_loop() {
	if (has_last_target) {
		aim_target(last_target);
	}
}
