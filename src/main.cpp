#include "seu-detect/Armor/ArmorDetector.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>

#define DEBUG

class DetectResult {
  public:
	int16_t center_x = -1; // 目标在图像中的 x 坐标; -1 未检测到目标, [0,32767]
	                       // 对应分数坐标 [0,1]
	int16_t center_y = -1; // 目标在图像中的 y 坐标; 同上
	int16_t t_x = -1;      // 目标 x 坐标 (mm), x 轴向右
	int16_t t_y = -1;      // 目标 y 坐标 (mm), y 轴向下
	int16_t t_z = -1;      // 目标 z 坐标 (mm), z 轴向前
	int16_t r_x = -1;      // 目标旋转向量 x 分量, 暂时为 -1
	int16_t r_y = -1;      // 目标旋转向量 y 分量, 暂时为 -1
	int16_t r_z = -1;      // 目标旋转向量 z 分量, 暂时为 -1
};

#ifdef DEBUG
double to_ms(std::chrono::duration<double, std::milli> t) {
	return t.count();
}
std::string to_detect_result(rm::ArmorDetector::ArmorFlag result) {
	switch (result) {
	case rm::ArmorDetector::ARMOR_NO:
		return "none";
	case rm::ArmorDetector::ARMOR_LOST:
		return "lost";
	case rm::ArmorDetector::ARMOR_GLOBAL:
		return "detected";
	case rm::ArmorDetector::ARMOR_LOCAL:
		return "tracking";
	default:
		assert(false);
	}
}
std::string to_armor_type(rm::ObjectType type) {
	switch (type) {
	case rm::UNKNOWN_ARMOR:
		return "unknown";
	case rm::SMALL_ARMOR:
		return "small";
	case rm::BIG_ARMOR:
		return "big";
	default:
		assert(false);
	}
}
#endif

std::string env(const std::string &var, const std::string &default_value) {
	char *val = getenv(var.c_str());
	return val == nullptr ? default_value : val;
}

int main() {
	// Open camera
	cv::VideoCapture cap{0};
	if (!cap.isOpened()) {
		std::cerr << "Couldn't open camera\n";
		std::abort();
	}
	const int frame_width = std::stoi(env("RM_WIDTH", "640"));
	const int frame_height = std::stoi(env("RM_HEIGHT", "480"));
	cap.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);

	// Initialize SEU armor detector
	rm::ArmorParam armor_param;
	rm::ArmorDetector armor_detector;
	armor_detector.init(armor_param);

	// Default enemy color
	const std::string default_enemy_color = env("RM_ENEMY_COLOR", "blue");
	if (default_enemy_color == "red") {
		armor_detector.setEnemyColor(rm::RED);
	} else if (default_enemy_color == "blue") {
		armor_detector.setEnemyColor(rm::BLUE);
	} else {
		std::cerr << "Unrecognized enemy color\n";
		std::abort();
	}

	cv::Mat frame;
	while (true) {

#ifdef DEBUG
		std::cerr << "----\n";
		auto t0 = std::chrono::high_resolution_clock::now();
#endif

		cap >> frame;

#ifdef DEBUG
		auto t1 = std::chrono::high_resolution_clock::now();
#endif

		DetectResult result;
		armor_detector.loadImg(frame);
		rm::ArmorDetector::ArmorFlag detect_result = armor_detector.detect();
		if (detect_result == rm::ArmorDetector::ARMOR_LOCAL ||
		    detect_result == rm::ArmorDetector::ARMOR_LOCAL) {
			std::vector<cv::Point2f> vertex = armor_detector.getArmorVertex();
			rm::ObjectType armor_type = armor_detector.getArmorType();

			int center_x = 0;
			int center_y = 0;
			for (auto &p : vertex) {
				center_x += p.x;
				center_y += p.y;
			}
			result.center_x = (((double)center_x) / 4 / frame_width) *
			                  std::numeric_limits<int16_t>::max();
			result.center_y = (((double)center_y) / 4 / frame_height) *
			                  std::numeric_limits<int16_t>::max();

			// TODO: calcute pose

#ifdef DEBUG
			std::vector<cv::Point> quad_points;
			for (auto &p : vertex) {
				quad_points.push_back(p);
			}
			cv::polylines(frame, quad_points, true, cv::Scalar(255, 255, 0));
			// clang-format off
			std::cerr << "armor_type: " << to_armor_type(armor_type) << "\n"
			          << "vertex0: (" << vertex[0].x << ", " << vertex[0].y << ")\n"
			          << "vertex1: (" << vertex[1].x << ", " << vertex[1].y << ")\n"
			          << "vertex2: (" << vertex[2].x << ", " << vertex[2].y << ")\n"
			          << "vertex3: (" << vertex[3].x << ", " << vertex[3].y << ")\n";
			// clang-format on

#endif
		}

#ifdef DEBUG
		auto t2 = std::chrono::high_resolution_clock::now();
#endif

#ifdef DEBUG
		std::cerr << "decode: " << to_ms(t1 - t0)
		          << "ms, detect: " << to_ms(t2 - t1)
		          << "ms, result: " << to_detect_result(detect_result)
		          << std::endl;
		cv::imshow("camera", frame);
		cv::waitKey(1);
#endif
	}
}
