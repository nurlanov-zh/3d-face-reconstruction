// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <thread>
#include <vector>

// This example assumes camera with depth and color
// streams, and direction lets you define the target stream
enum class direction
{
	to_depth,
	to_color
};

typedef pcl::PointXYZRGB P_pcl;
typedef pcl::PointCloud<P_pcl> point_cloud;
typedef point_cloud::Ptr ptr_cloud;

using std::condition_variable;
using std::mutex;
using std::queue;
using std::thread;
using std::unique_lock;
using std::vector;

class WorkQueue
{
	condition_variable work_available;
	mutex work_mutex;
	queue<std::tuple<rs2::points, rs2::video_frame, int>> work;

   public:
	void push_work(std::tuple<rs2::points, rs2::video_frame, int> item)
	{
		unique_lock<mutex> lock(work_mutex);

		bool was_empty = work.empty();
		work.push(item);

		lock.unlock();

		if (was_empty)
		{
			work_available.notify_one();
		}
	}

	size_t size()
	{
		unique_lock<mutex> lock(work_mutex);
		return work.size();
	}

	std::tuple<rs2::points, rs2::video_frame, int> wait_and_pop()
	{
		unique_lock<mutex> lock(work_mutex);
		while (work.empty())
		{
			work_available.wait(lock);
		}

		return work.front();
	}

	void pop()
	{
		unique_lock<mutex> lock(work_mutex);

		work.pop();
	}
};

WorkQueue work_queue;

std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(
	rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
	const int w = texture.get_width(), h = texture.get_height();

	// convert normals [u v] to basic coords [x y]
	int x = std::min(std::max(int(texcoords.u * w + .5f), 0), w - 1);
	int y = std::min(std::max(int(texcoords.v * h + .5f), 0), h - 1);

	int idx =
		x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
	const auto texture_data =
		reinterpret_cast<const uint8_t*>(texture.get_data());
	return std::tuple<uint8_t, uint8_t, uint8_t>(
		texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
}

ptr_cloud points_to_pcl(const rs2::points& points,
						const rs2::video_frame& color)
{
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	ptr_cloud cloud(new point_cloud);

	// Config of PCL Cloud object
	cloud->width = 1920;
	cloud->height = 1080;
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto tex_coords = points.get_texture_coordinates();
	auto vertices = points.get_vertices();

// Iterating through all points and setting XYZ coordinates
// and RGB values
#pragma omp parallel for
	for (int i = 0; i < points.size(); ++i)
	{
		cloud->points[i].x = vertices[i].x;
		cloud->points[i].y = vertices[i].y;
		cloud->points[i].z = vertices[i].z;

		std::tuple<uint8_t, uint8_t, uint8_t> current_color;
		current_color = get_texcolor(color, tex_coords[i]);

		// Reversed order- 2-1-0 because of BGR model used in camera
		cloud->points[i].r = std::get<2>(current_color);
		cloud->points[i].g = std::get<1>(current_color);
		cloud->points[i].b = std::get<0>(current_color);
	}

	return cloud;
}

void catch_image()
{
	// Create and initialize GUI related objects
	rs2::colorizer c;  // Helper to colorize depth images

	// Create a pipeline to easily configure and start the camera
	rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_DEPTH);
	cfg.enable_stream(RS2_STREAM_COLOR);
	pipe.start(cfg);

	// Define two align objects. One will be used to align
	// to depth viewport and the other to color.
	// Creating align object is an expensive operation
	// that should not be performed in the main loop
	rs2::align align_to_depth(RS2_STREAM_DEPTH);
	rs2::align align_to_color(RS2_STREAM_COLOR);

	direction dir = direction::to_depth;  // Alignment direction
	int id = 0;
	while (id < 400)  // Application still alive?
	{
		// Using the align object, we block the application until a frameset is
		// available
		rs2::frameset frameset = pipe.wait_for_frames();

		if (dir == direction::to_color)
		{
			// Align all frames to depth viewport
			frameset = align_to_depth.process(frameset);
		}
		else
		{
			// Align all frames to color viewport
			frameset = align_to_color.process(frameset);
		}

		// With the aligned frameset we proceed as usual
		auto depth = frameset.get_depth_frame();
		auto color = frameset.get_color_frame();

		const int w = depth.as<rs2::video_frame>().get_width();
		const int h = depth.as<rs2::video_frame>().get_height();

		cv::Mat depth_img(cv::Size(w, h), CV_16UC1, (void*)depth.get_data(),
						  cv::Mat::AUTO_STEP);
		cv::Mat rgb_img(cv::Size(w, h), CV_8UC3, (void*)color.get_data(),
						cv::Mat::AUTO_STEP);

		cv::Mat bgr_img;
		cv::cvtColor(rgb_img, bgr_img, cv::COLOR_BGR2RGB);
		cv::imwrite(std::to_string(id) + "_rgb.png", bgr_img);
		rs2::pointcloud pc;
		pc.map_to(color);
		auto points = pc.calculate(depth);
		std::cout << "processing " << id << std::endl;

		work_queue.push_work(std::make_tuple(points, color, id));
		id++;
	}
}

int main(int argc, char* argv[]) try
{
	vector<thread> producers;

	auto producer = [&]() { catch_image(); };

	producers.push_back(std::thread(producer));

	std::thread consumer([&]() {
		while (true)
		{
			auto work_to_do = work_queue.wait_and_pop();
			ptr_cloud cloud = points_to_pcl(std::get<0>(work_to_do), std::get<1>(work_to_do));

			pcl::io::savePCDFileBinary(
				std::to_string(std::get<2>(work_to_do)) + "_depth.pcd",
				*cloud);
			work_queue.pop();
			std::cout << "processed " << std::get<2>(work_to_do) << " frame"
					  << std::endl;
			std::cout << "queue size " << work_queue.size() << std::endl;
		}
	});

	std::for_each(producers.begin(), producers.end(),
				  [](thread& p) { p.join(); });

	consumer.join();

	return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "("
			  << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}