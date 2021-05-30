#include "lookout_adapter.hpp"
#include <std_msgs/String.h>

std::vector<std::string> split_str(const std::string &str,
                                   const std::string &deli) {
	std::vector<std::string> result;
	std::string::size_type pos_begin = 0;
	for (;;) {
		auto pos_end = str.find(deli, pos_begin);
		if (pos_end == std::string::npos) {
			result.push_back(str.substr(pos_begin));
			break;
		} else {
			result.push_back(str.substr(pos_begin, pos_end - pos_begin));
			pos_begin = pos_end + 1;
		}
	}
	return result;
}

std::vector<Vehicle> deserialize_lookout_data(const std::string &str) {
	std::vector<Vehicle> result;

	auto split = split_str(str, ";");
	for (size_t i = 0; (i + 2) < split.size(); i += 3) {
		Vehicle v;
		if (split[i] == "red") {
			v.color = VehicleColor::RED;
		} else if (split[i] == "blue") {
			v.color = VehicleColor::BLUE;
		} else if (split[i] == "unknown") {
			v.color = VehicleColor::UNKNOWN;
		} else {
			throw std::runtime_error("malformed lookout data: color " +
			                         split[i]);
		}
		v.x = std::stod(split[i + 1]);
		v.y = std::stod(split[i + 2]);
		result.push_back(v);
	}

	return result;
}

LookoutAdapter::LookoutAdapter() {
	ros::NodeHandle nh;

	lookout_sub = nh.subscribe<std_msgs::String>(
	    "lookout_data", 1, [this](const std_msgs::String::ConstPtr &msg) {
		    lookout_data = deserialize_lookout_data(msg->data);
		    lookout_data_recv_time = ros::Time::now();
	    });
}

ros::Duration lookout_data_expiry_duration{0.5};

std::vector<Vehicle> LookoutAdapter::get_cars() {
	if (ros::Time::now() - lookout_data_recv_time >
	    lookout_data_expiry_duration) {
		return {};
	}
	return lookout_data;
}

std::optional<VehicleColor> get_my_vehicle_color() {
	std::string team_color_str;
	ros::param::getCached("team_color", team_color_str);
	if (team_color_str == "red") {
		return VehicleColor::RED;
	} else if (team_color_str == "blue") {
		return VehicleColor::BLUE;
	} else {
		ROS_WARN("unrecognized team_color param: %s", team_color_str.c_str());
		return std::nullopt;
	}
}
