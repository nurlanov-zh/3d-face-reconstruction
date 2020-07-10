#include <dlib/opencv/cv_image.h>
#include <landmark_detection/face_landmark_detection.h>

namespace landmark_detection
{
LandmarkDetection::LandmarkDetection(const std::string& shapePredictorPath)
{
	std::ifstream file(shapePredictorPath);
	dlib::deserialize(predictor_, file);
	detector_ = dlib::get_frontal_face_detector();

	consoleLog_ = spdlog::get("console");
	errLog_ = spdlog::get("stderr");
}

std::vector<dlib::point> LandmarkDetection::detect(const cv::Mat& image)
{
	std::vector<dlib::point> landmarks;

	try
	{
		dlib::array2d<dlib::bgr_pixel> dlibImage;
		dlib::assign_image(dlibImage, dlib::cv_image<dlib::bgr_pixel>(image));

		std::vector<dlib::rectangle> dets = detector_(dlibImage);

		if (dets.size() == 0)
		{
			return {};
		}

		dlib::rectangle biggestRect(0, 0);

		for (size_t j = 0; j < dets.size(); ++j)
		{
			// find the biggest rect on the image
			if (dets[j].area() > biggestRect.area())
			{
				biggestRect = dets[j];
			}
		}

		dlib::full_object_detection shape = predictor_(dlibImage, biggestRect);
		for (size_t k = 0; k < shape.num_parts(); k++)
		{
			landmarks.emplace_back(shape.part(k));
		}
	}
	catch (std::exception&)
	{
		errLog_->error(
			"Exception is thrown during facial landmarks detection!");
	}

	return landmarks;
}
}  // namespace landmark_detection