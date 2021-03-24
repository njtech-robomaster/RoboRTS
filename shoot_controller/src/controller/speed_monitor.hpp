#pragma once

#include <vector>

class SpeedMonitor {
  public:
	double update_velocity(double new_bullet_velocity);

  private:
	std::vector<double> velocity_list;
};
