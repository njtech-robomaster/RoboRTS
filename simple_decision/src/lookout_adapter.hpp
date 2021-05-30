#pragma once

#include <geometry_msgs/PointStamped.h>
#include <optional>
#include <ros/ros.h>

enum class VehicleColor { RED, BLUE, UNKNOWN };
class Vehicle {
  public:
	VehicleColor color;
	double x;
	double y;
};

std::optional<VehicleColor> get_my_vehicle_color();

class LookoutAdapter {
  public:
	LookoutAdapter();

	std::vector<Vehicle> get_cars();

  private:
	ros::Subscriber lookout_sub;
	ros::Time lookout_data_recv_time;
	std::vector<Vehicle> lookout_data;
};
