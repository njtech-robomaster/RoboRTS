#include "serial.h"
#include "seu-detect/Armor/ArmorDetector.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>

#define DEBUG

class DetectResult {
  public:
	float center_x = NAN;
	float center_y = NAN;
	float t_x = NAN;
	float t_y = NAN;
	float t_z = NAN;
	float r_x = NAN;
	float r_y = NAN;
	float r_z = NAN;
};

#ifdef DEBUG
double to_ms(std::chrono::duration<double, std::milli> t) {
	return t.count();
}
std::string to_detect_result(int result) {
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
#endif

std::string env(const std::string &var, const std::string &default_value) {
	char *val = getenv(var.c_str());
	return val == nullptr ? default_value : val;
}

void put_float_be(char *&ptr, const float &val) {
	static_assert(sizeof(float) == sizeof(uint32_t));
	uint32_t fbits;
	memcpy(&fbits, &val, sizeof(float));
	ptr[0] = (char)((fbits >> 24) & 0xff);
	ptr[1] = (char)((fbits >> 16) & 0xff);
	ptr[2] = (char)((fbits >> 8) & 0xff);
	ptr[3] = (char)((fbits >> 0) & 0xff);
	ptr += 4;
}
void send_result(SerialPort &serial, const DetectResult &result) {
	static char buf[34];
	char *ptr = buf;
	*(ptr++) = 0x22;
	put_float_be(ptr, result.center_x);
	put_float_be(ptr, result.center_y);
	put_float_be(ptr, result.t_x);
	put_float_be(ptr, result.t_y);
	put_float_be(ptr, result.t_z);
	put_float_be(ptr, result.r_x);
	put_float_be(ptr, result.r_y);
	put_float_be(ptr, result.r_z);
	*(ptr++) = 0x33;

#ifdef DEBUG
	assert(ptr == buf + sizeof(buf));
	for (size_t i = 0; i < sizeof(buf); i++) {
		std::cerr << std::setfill('0') << std::setw(2) << std::hex
		          << (0xff & (unsigned int)buf[i]);
	}
	std::cerr << std::endl;
#endif

	serial.send(buf, sizeof(buf));
}

int main() {
	// Initialize serial port
	SerialPort serial{"/dev/ttyACM0"};

	// Open camera
	cv::VideoCapture cap{std::stoi(env("RM_CAMERA", "0"))};
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
	armor_detector.setEnemyColor(
	    rm::BLUE); // TODO Retrive color from serial port

	cv::Mat frame;
	while (true) {

#ifdef DEBUG
		auto t0 = std::chrono::high_resolution_clock::now();
#endif

		cap >> frame;

#ifdef DEBUG
		auto t1 = std::chrono::high_resolution_clock::now();
#endif

		DetectResult result;
		armor_detector.loadImg(frame);
		int detect_result = armor_detector.detect();
		if (detect_result == rm::ArmorDetector::ARMOR_LOCAL ||
		    detect_result == rm::ArmorDetector::ARMOR_LOCAL) {
			std::vector<cv::Point2f> vertex = armor_detector.getArmorVertex();
			int armor_type = armor_detector.getArmorType();

			result.center_x = 0;
			result.center_y = 0;
			for (auto &p : vertex) {
				result.center_x += p.x;
				result.center_y += p.y;
			}
			result.center_x /= 4;
			result.center_x /= frame_width;
			result.center_y /= 4;
			result.center_y /= frame_height;

#ifdef DEBUG
			std::vector<cv::Point> quad_points;
			for (auto &p : vertex) {
				quad_points.push_back(p);
			}
			cv::polylines(frame, quad_points, true, cv::Scalar(255, 255, 0));
			std::cerr << "armor_type: " << armor_type << std::endl;
#endif
		}

		send_result(serial, result);

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
