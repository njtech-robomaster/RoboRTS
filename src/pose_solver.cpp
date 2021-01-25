#include "pose_solver.h"
#include <stdexcept>

const std::vector<cv::Point3f> OBJECT_POINTS_ARMOR_BIG = {

    {-105, -30, 0}, // top-left
    {105, -30, 0},  // top-right
    {105, 30, 0},   // bottom-right
    {-105, 30, 0}   // bottom-left
};

const std::vector<cv::Point3f> OBJECT_POINTS_ARMOR_SMALL = {
    {-60, -68, 0}, // top-left
    {60, -68, 0},  // top-right
    {60, 68, 0},   // bottom-right
    {-60, 68, 0}   // bottom-left
};

/*
const std::vector<cv::Point3f> OBJECT_POINTS_ARMOR_SMALL = {
    {-60, -62.5, 0}, // top-left
    {60, -62.5, 0},  // top-right
    {60, 62.5, 0},   // bottom-right
    {-60, 62.5, 0}   // bottom-left
};
*/

const std::vector<cv::Point3f> &get_object_points(rm::ObjectType armor_type) {
	switch (armor_type) {
	case rm::BIG_ARMOR:
		return OBJECT_POINTS_ARMOR_BIG;
	case rm::SMALL_ARMOR:
		return OBJECT_POINTS_ARMOR_SMALL;
	default:
		assert(false);
	}
}

void PoseSolver::load_parameters(const std::string &parameters_file) {
	cv::FileStorage xml(parameters_file, cv::FileStorage::READ);
	if (!xml.isOpened()) {
		throw std::runtime_error("Couldn't read parameters file " +
		                         parameters_file);
	}
	xml["camera_intrinsic"] >> camera_intrinsic_mat;
	xml["distortion_coefficients"] >> distortion_coefficients_mat;
}

PoseInfo PoseSolver::solve(const std::vector<cv::Point2f> &object_points,
                           rm::ObjectType armor_type) {
	cv::Mat r_vec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat t_vec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::solvePnP(get_object_points(armor_type), object_points,
	             camera_intrinsic_mat, distortion_coefficients_mat, r_vec,
	             t_vec, false, cv::SOLVEPNP_ITERATIVE);
	PoseInfo result;
	result.r_x = r_vec.at<double>(0, 0);
	result.r_y = r_vec.at<double>(1, 0);
	result.r_z = r_vec.at<double>(2, 0);
	result.t_x = t_vec.at<double>(0, 0);
	result.t_y = t_vec.at<double>(1, 0);
	result.t_z = t_vec.at<double>(2, 0);
	return result;
}
