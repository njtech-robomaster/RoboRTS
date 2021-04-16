#pragma once

#include "kcfcpp/kcftracker.hpp"
#include "seu-detect/Armor/ArmorDetector.h"
#include <atomic>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

class DetectResult {
  public:
	bool is_estimated;
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

	KCFTracker *kcf_tracker;
	cv::Mat last_detected_color_frame;
	cv::Rect last_detected_roi;
	std::chrono::high_resolution_clock::time_point last_detected_time;

	void reset_kcf_tracking();
};
