#pragma once

#include "seu-detect/General/General.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class PoseInfo {
  public:
	double t_x = NAN;
	double t_y = NAN;
	double t_z = NAN;
	double r_x = NAN;
	double r_y = NAN;
	double r_z = NAN;
};

class PoseSolver {
  private:
	cv::Mat camera_intrinsic_mat;
	cv::Mat distortion_coefficients_mat;

  public:
	void load_parameters(const std::string &parameters_file);
	PoseInfo solve(const std::vector<cv::Point2f> &object_points,
	               rm::ObjectType armor_type);
};
