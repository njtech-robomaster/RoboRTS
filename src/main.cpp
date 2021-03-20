#include "cv-helpers.hpp"
#include "seu-detect/Armor/ArmorDetector.h"
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>

const float shrink_factor = .3;

#define DEBUG

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
void draw_quad(cv::Mat display_mat, std::vector<cv::Point2f> verticies,
               cv::Scalar color) {
	std::vector<cv::Point> quad_points;
	for (auto &p : verticies) {
		quad_points.push_back(p);
	}
	cv::polylines(display_mat, quad_points, true, color);
}
#endif

std::string env(const std::string &var, const std::string &default_value) {
	char *val = getenv(var.c_str());
	return val == nullptr ? default_value : val;
}

uint16_t mean_distance_within_quad(const cv::Mat &depth_mat,
                                   const std::vector<cv::Point2f> verticies) {
	std::vector<cv::Point> pts;
	for (auto &point : verticies) {
		pts.push_back({(int)std::round(point.x), (int)std::round(point.y)});
	}

	int min_x = std::max(std::min({pts[0].x, pts[1].x, pts[2].x, pts[3].x}), 0);
	int max_x = std::min(std::max({pts[0].x, pts[1].x, pts[2].x, pts[3].x}),
	                     depth_mat.cols - 1);
	int min_y = std::max(std::min({pts[0].y, pts[1].y, pts[2].y, pts[3].y}), 0);
	int max_y = std::min(std::max({pts[0].y, pts[1].y, pts[2].y, pts[3].y}),
	                     depth_mat.rows - 1);

	cv::Mat1b mask(depth_mat.rows, depth_mat.cols, uchar(0));
	cv::fillConvexPoly(mask, pts, cv::Scalar(255));

	uint64_t sum = 0;
	uint32_t samples = 0;
	for (int y = min_y; y <= max_y; y++) {
		for (int x = min_x; x <= max_x; x++) {
			if (mask.at<uchar>(y, x) == 0)
				continue;
			uint16_t distance = depth_mat.at<uint16_t>(y, x);
			if (distance == 0)
				continue;
			samples++;
			sum += distance;
		}
	}
	return samples == 0 ? 0 : sum / samples;
}

int main() {
	// Open pipeline
	rs2::pipeline pipeline;
	rs2::config rs_config;
	rs_config.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8);
	rs_config.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16);

	// Get color camera intrinsics
	auto pipeline_profile = pipeline.start(rs_config);
	auto color_stream = pipeline_profile.get_stream(RS2_STREAM_COLOR)
	                        .as<rs2::video_stream_profile>();
	auto color_intrinsics = color_stream.get_intrinsics();

	// Setup depth-to-color align
	rs2::align align_to_color{RS2_STREAM_COLOR};

	// Initialize SEU armor detector
	rm::ArmorParam armor_param;
	rm::ArmorDetector armor_detector;
	armor_detector.init(armor_param);
	armor_detector.setEnemyColor(rm::BLUE);

	while (true) {

#ifdef DEBUG
		std::cerr << "----\n";
		auto t0 = std::chrono::high_resolution_clock::now();
#endif

		rs2::frameset frames = pipeline.wait_for_frames();
		frames = align_to_color.process(frames);
		rs2::video_frame color_frame = frames.get_color_frame();
		rs2::depth_frame depth_frame = frames.get_depth_frame();
		cv::Mat color_mat = frame_to_mat(color_frame);
		cv::Mat depth_mat = frame_to_mat(depth_frame);

#ifdef DEBUG
		cv::Mat display_mat = color_mat.clone();
		auto t1 = std::chrono::high_resolution_clock::now();
#endif

		armor_detector.loadImg(color_mat);
		rm::ArmorDetector::ArmorFlag detect_result = armor_detector.detect();
		if (detect_result == rm::ArmorDetector::ARMOR_LOCAL ||
		    detect_result == rm::ArmorDetector::ARMOR_LOCAL) {
			std::vector<cv::Point2f> verticies =
			    armor_detector.getArmorVertex();
			rm::ObjectType armor_type = armor_detector.getArmorType();

			// calculate 2d center
			cv::Point2f center_point =
			    (verticies[0] + verticies[1] + verticies[2] + verticies[3]) / 4;

			// calculate shrinked quad
			std::vector<cv::Point2f> shrinked_verticies;
			for (auto &origin : verticies) {
				cv::Point2f shrinked =
				    (origin - center_point) * (1 - shrink_factor) +
				    center_point;
				shrinked_verticies.push_back(shrinked);
			}

			// calculate distance
			float distance =
			    mean_distance_within_quad(depth_mat, shrinked_verticies) *
			    depth_frame.get_units();

			// calculate 3d position
			float pos_2d[] = {center_point.x, center_point.y};
			float pos_3d[3];
			rs2_deproject_pixel_to_point(pos_3d, &color_intrinsics, pos_2d,
			                             distance);
			cv::Point3f center_position{pos_3d[0], pos_3d[1], pos_3d[2]};

#ifdef DEBUG
			draw_quad(display_mat, verticies, {255, 255, 0});
			draw_quad(display_mat, shrinked_verticies, {0, 255, 0});
			// clang-format off
			std::cerr << "armor_type: " << to_armor_type(armor_type) << "\n"
			          << "vertex0: (" << verticies[0].x << ", " << verticies[0].y << ")\n"
			          << "vertex1: (" << verticies[1].x << ", " << verticies[1].y << ")\n"
			          << "vertex2: (" << verticies[2].x << ", " << verticies[2].y << ")\n"
			          << "vertex3: (" << verticies[3].x << ", " << verticies[3].y << ")\n"
					  << "distance: " << distance << "\n"
					  << "position: (" <<  center_position.x <<", "<<  center_position.y <<", "<<  center_position.z <<")\n";
			// clang-format on
#endif
		}

#ifdef DEBUG
		auto t2 = std::chrono::high_resolution_clock::now();

		std::cerr << "decode: " << to_ms(t1 - t0)
		          << "ms, detect: " << to_ms(t2 - t1)
		          << "ms, result: " << to_detect_result(detect_result)
		          << std::endl;
		cv::imshow("camera", display_mat);
		cv::waitKey(1);
#endif
	}
}
