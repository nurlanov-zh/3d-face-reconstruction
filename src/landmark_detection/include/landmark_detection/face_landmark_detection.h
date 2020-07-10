#pragma once

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/image_processing.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>

#include <common/data_types.h>

#include <opencv2/opencv.hpp>

#include <iostream>

namespace landmark_detection
{
class LandmarkDetection
{
   public:
	LandmarkDetection(const std::string& shapePredictorPath);

	std::vector<dlib::point> detect(const cv::Mat& image);

   private:
	std::shared_ptr<spdlog::logger> consoleLog_;
	std::shared_ptr<spdlog::logger> errLog_;

	dlib::frontal_face_detector detector_;
	dlib::shape_predictor predictor_;
};

// std::vector<Eigen::Vector3f> cloud_landmarks(
// 	const std::string& cloud_path, const std::vector<dlib::point>&
// image_landmarks);

}  // namespace landmark_detection