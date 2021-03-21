#pragma once

#include "seu-detect/Armor/ArmorDetector.h"
#include <atomic>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

class DetectResult {
  public:
	std::vector<cv::Point2f> vertices;
	cv::Point3d position;
};

class RSArmorDetector {
  public:
	RSArmorDetector(cv::Size2i color_resolution, cv::Size2i depth_resolution);
	bool poll();
	bool detect(DetectResult &result);

  private:
	rs2::pipeline pipeline;
	rs2_intrinsics color_intrinsics;
	rs2::align align_to_color;
	rm::ArmorDetector seu_armor_detector;
	rs2::frameset frames;
	rs2::context ctx;
	std::atomic_bool is_disconnected;
};
