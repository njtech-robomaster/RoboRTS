#include "speed_monitor.hpp"

double SpeedMonitor::update_velocity(double new_bullet_velocity) {
	velocity_list.push_back(new_bullet_velocity);

	return new_bullet_velocity;
}
