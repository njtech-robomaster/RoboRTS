#include "detector.hpp"
#include "cv-helpers.hpp"
#include <librealsense2/rsutil.h>
#include <ros/console.h>
#include <ros/ros.h>

const float shrink_factor = .3;

RSArmorDetector::RSArmorDetector(cv::Size2i color_resolution,
                                 cv::Size2i depth_resolution)
    : align_to_color{RS2_STREAM_COLOR}, is_disconnected{false} {
	rs2::config rs_config;
	rs_config.enable_stream(RS2_STREAM_COLOR, color_resolution.width,
	                        color_resolution.height, RS2_FORMAT_BGR8);
	rs_config.enable_stream(RS2_STREAM_DEPTH, depth_resolution.width,
	                        depth_resolution.height, RS2_FORMAT_Z16);
	auto pipeline_profile = pipeline.start(rs_config);
	auto color_stream = pipeline_profile.get_stream(RS2_STREAM_COLOR)
	                        .as<rs2::video_stream_profile>();
	color_intrinsics = color_stream.get_intrinsics();

	auto device = pipeline_profile.get_device();
	ctx.set_devices_changed_callback([=](rs2::event_information &info) {
		if (info.was_removed(device)) {
			is_disconnected = true;
			ROS_ERROR("RealSense is disconnected.");
		}
	});

	rm::ArmorParam armor_param;
	seu_armor_detector.init(armor_param);
}

bool RSArmorDetector::poll() {
	if (pipeline.poll_for_frames(&frames)) {
		frames = align_to_color.process(frames);
		return true;
	}
	if (is_disconnected) {
		throw rs2::error{"Device disconnected"};
	}
	return false;
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

void setup_target_color(rm::ArmorDetector &seu_armor_detector) {
	std::string val;
	ros::param::getCached("team_color", val);
	if (val == "red") {
		seu_armor_detector.setEnemyColor(rm::BLUE);
	} else if (val == "blue") {
		seu_armor_detector.setEnemyColor(rm::RED);
	} else {
		ROS_WARN("Unrecognized team_color param value: %s", val.c_str());
	}
}

bool RSArmorDetector::detect(DetectResult &result) {
	auto color_frame = frames.get_color_frame();
	auto depth_frame = frames.get_depth_frame();
	auto color_mat = frame_to_mat(color_frame);
	auto depth_mat = frame_to_mat(depth_frame);

	setup_target_color(seu_armor_detector);
	seu_armor_detector.loadImg(color_mat);
	if (seu_armor_detector.detect() != rm::ArmorDetector::ARMOR_LOCAL)
		return false;

	auto verticies = seu_armor_detector.getArmorVertex();

	// calculate 2d center
	cv::Point2f center_point =
	    (verticies[0] + verticies[1] + verticies[2] + verticies[3]) / 4;

	// calculate shrinked quad
	std::vector<cv::Point2f> shrinked_verticies;
	for (auto &origin : verticies) {
		cv::Point2f shrinked =
		    (origin - center_point) * (1 - shrink_factor) + center_point;
		shrinked_verticies.push_back(shrinked);
	}

	// calculate distance
	uint16_t distance_raw =
	    mean_distance_within_quad(depth_mat, shrinked_verticies);
	if (distance_raw == 0) // no depth info is available
		return false;
	float distance = distance_raw * depth_frame.get_units();

	// calculate 3d position
	float pos_2d[] = {center_point.x, center_point.y};
	float pos_3d[3];
	rs2_deproject_pixel_to_point(pos_3d, &color_intrinsics, pos_2d, distance);
	cv::Point3f center_position{pos_3d[0], pos_3d[1], pos_3d[2]};

	result.vertices = verticies;
	result.position = center_position;
	return true;
}
