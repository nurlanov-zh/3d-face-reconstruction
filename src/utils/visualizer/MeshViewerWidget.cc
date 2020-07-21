/* ========================================================================= *
 *                                                                           *
 *                               OpenMesh                                    *
 *           Copyright (c) 2001-2015, RWTH-Aachen University                 *
 *           Department of Computer Graphics and Multimedia                  *
 *                          All rights reserved.                             *
 *                            www.openmesh.org                               *
 *                                                                           *
 *---------------------------------------------------------------------------*
 * This file is part of OpenMesh.                                            *
 *---------------------------------------------------------------------------*
 *                                                                           *
 * Redistribution and use in source and binary forms, with or without        *
 * modification, are permitted provided that the following conditions        *
 * are met:                                                                  *
 *                                                                           *
 * 1. Redistributions of source code must retain the above copyright notice, *
 *    this list of conditions and the following disclaimer.                  *
 *                                                                           *
 * 2. Redistributions in binary form must reproduce the above copyright      *
 *    notice, this list of conditions and the following disclaimer in the    *
 *    documentation and/or other materials provided with the distribution.   *
 *                                                                           *
 * 3. Neither the name of the copyright holder nor the names of its          *
 *    contributors may be used to endorse or promote products derived from   *
 *    this software without specific prior written permission.               *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED *
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A           *
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER *
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,  *
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,       *
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR        *
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    *
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING      *
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS        *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.              *
 *                                                                           *
 * ========================================================================= */

#define OPENMESHAPPS_MESHVIEWERWIDGET_CC

//== INCLUDES =================================================================

#include "MeshViewerWidget.hh"
#include <unordered_set>

//== IMPLEMENTATION ==========================================================

/// default constructor

const std::string DETECTION_MODEL_PATH =
	"../data/detector/shape_predictor_68_face_landmarks.dat";

constexpr int32_t WIDTH = 1280;
constexpr int32_t BAR = 100;
constexpr int32_t HEIGHT = 720;
const std::unordered_set<int32_t> landmarksIdsProcrustes = {
	0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16,
	17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33,
	34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
	51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67};

MeshViewerWidget::MeshViewerWidget(bool sequence,
								   const OpenMesh::IO::Options& opt,
								   QWidget* parent)
	: MeshViewerWidgetT<common::Mesh>(parent), seq_(sequence)
{
	outputVideo_.open("video.avi", 0, 2, cv::Size(WIDTH, HEIGHT - BAR), true);
	consoleLog_ = spdlog::get("console");
	errLog_ = spdlog::get("stderr");

	dataReader_.reset(new utils::DataReader("../data", opt, 6));

	common::Mesh& neutralMesh = dataReader_->getNeutralMesh();
	if (!neutralMesh.has_vertex_normals())
	{
		neutralMesh.request_vertex_normals();
	}

	matching::refinement::NRICPParams params;
	params.numOfEdges = neutralMesh.n_edges();
	params.numOfVertices = neutralMesh.n_vertices();
	params.numOfLandmarks = 68;
	params.betaInit = 1;
	params.alphaInit = 1000;
	params.alphaMin = 100;
	params.numOfOuterIterations = 10;
	nricp_.reset(new matching::refinement::NRICP(params));

	Eigen::Vector3d trans = {-0.00356676848605, 0.0257014129311,
							 0.00136031582952};
	const Eigen::Quaterniond q = Eigen::Quaterniond(
		0.999997079372, 0.00196678028442, -0.0010471905116, -0.000916811462957);
	imageToDepth_ = Sophus::SE3d(q.normalized().toRotationMatrix(), trans);

	next_frame();
}

void MeshViewerWidget::saveImage(const std::string& filename)
{
	cv::Mat pixels(HEIGHT - BAR, WIDTH, CV_8UC3);
	glReadPixels(0, 0, WIDTH, HEIGHT - BAR, GL_RGB, GL_UNSIGNED_BYTE,
				 pixels.data);
	cv::Mat image(HEIGHT - BAR, WIDTH, CV_8UC3);

	for (int y = 0; y < HEIGHT - BAR; y++)
	{
		for (int x = 0; x < WIDTH; x++)
		{
			image.at<cv::Vec3b>(y, x)[2] =
				pixels.at<cv::Vec3b>(HEIGHT - BAR - y - 1, x)[0];
			image.at<cv::Vec3b>(y, x)[1] =
				pixels.at<cv::Vec3b>(HEIGHT - BAR - y - 1, x)[1];
			image.at<cv::Vec3b>(y, x)[0] =
				pixels.at<cv::Vec3b>(HEIGHT - BAR - y - 1, x)[2];
		}
	}

	cv::imwrite(filename, image);

	outputVideo_ << image;
}

void MeshViewerWidget::next_frame()
{
	static int id = 0;
	common::Mesh neutralMesh = dataReader_->getNeutralMesh();
	if (!neutralMesh.has_vertex_normals())
	{
		neutralMesh.request_vertex_normals();
	}

	if (seq_)
	{
		landmark_detection::LandmarkDetection lmd(DETECTION_MODEL_PATH);
		const auto& assignedLandmarks = dataReader_->getAssignedLandmarks();
		if (dataReader_->isNextRealSenseExists())
		{
			const auto nextRgbd = dataReader_->nextRealSense();
			if (!nextRgbd.has_value())
			{
				spdlog::get("stderr")->error("No rgbd sequence! Exit");
				return;
			}

			const auto& landmarks = lmd.detect(nextRgbd.value().first);

			std::vector<common::Vec2i> correspondences;
			std::vector<common::Vec2i> procrustesCorrespondences;

			common::Mesh mesh;
			mesh.request_face_normals();
			mesh.request_face_colors();
			mesh.request_vertex_normals();
			mesh.request_vertex_colors();
			mesh.request_vertex_texcoords2D();

			cv::Mat image = nextRgbd.value().first;
			// dirty but who cares
			std::vector<std::pair<int32_t, pcl::PointXYZRGB>> points;
			for (size_t i = 0; i < landmarks.size(); ++i)
			{
				image.at<cv::Vec3b>(landmarks[i](1), landmarks[i](0)) = {255, 0,
																		 0};
				const auto lm3d = nextRgbd.value().second->at(landmarks[i](0),
															  landmarks[i](1));
				if (!std::isnan(lm3d.x) && !std::isnan(lm3d.y) &&
					!std::isnan(lm3d.z))
				{
					points.push_back(std::make_pair(i, lm3d));
				}
			}
			cv::imwrite("/tmp/image_lm_" + std::to_string(id) + ".png", image);

			pcl::PointXYZRGB mean;

			for (size_t i = 0; i < points.size(); ++i)
			{
				const auto point = points[i].second;
				mean.x += point.x;
				mean.y += point.y;
				mean.z += point.z;
			}
			mean.x /= points.size();
			mean.y /= points.size();
			mean.z /= points.size();

			for (size_t i = 0; i < points.size(); ++i)
			{
				if ((mean.x - points[i].second.x) *
							(mean.x - points[i].second.x) +
						(mean.y - points[i].second.y) *
							(mean.y - points[i].second.y) +
						(mean.z - points[i].second.z) *
							(mean.z - points[i].second.z) <
					0.1)
				{
					const common::Mesh::VertexHandle& handle =
						mesh.add_vertex(common::Mesh::Point(
							points[i].second.x, -points[i].second.y,
							-points[i].second.z));

					if (landmarksIdsProcrustes.find(points[i].first) !=
						landmarksIdsProcrustes.end())
					{
						procrustesCorrespondences.push_back(
							{assignedLandmarks[i], handle.idx()});
					}
					correspondences.push_back(
						{assignedLandmarks[i], handle.idx()});
					mesh.set_color(handle, {255, 0, 0});
					neutralMesh.set_color(
						common::Mesh::VertexHandle(assignedLandmarks[i]),
						{255, 0, 0});
				}
			}

			for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it =
					 nextRgbd.value().second->begin();
				 it != nextRgbd.value().second->end(); ++it)
			{
				// if (it->x < 0.5 && it->x > -0.5 && it->y > -2.0 &&
				// 	it->y < 2.0 && it->z > 0 && it->z < 2.5)
				{
					uint32_t rgb = *reinterpret_cast<uint32_t*>(&(it->rgb));
					uint8_t r = (rgb >> 16) & 0x0000ff;
					uint8_t g = (rgb >> 8) & 0x0000ff;
					uint8_t b = (rgb)&0x0000ff;
					const auto& handle = mesh.add_vertex(
						common::Mesh::Point(it->x, -it->y, -it->z));
					mesh.set_color(handle, {b, g, r});
				}
			}
			mesh.update_vertex_normals();
			mesh.update_face_normals();

			calculateFace(neutralMesh, mesh, procrustesCorrespondences,
						  correspondences);
		}
	}
	else
	{
		const auto& correspondences = dataReader_->getCorrespondences();
		common::Mesh& kinectMesh = dataReader_->getKinectMesh();
		if (!kinectMesh.has_vertex_normals())
		{
			kinectMesh.request_vertex_normals();
		}

		calculateFace(neutralMesh, kinectMesh, correspondences,
					  correspondences);

		kinectMesh.release_vertex_normals();
	}
	saveImage(std::to_string(id++) + ".png");
}

void MeshViewerWidget::play()
{
	while (dataReader_->isNextRealSenseExists())
	{
		next_frame();
	}
	outputVideo_.release();
}

common::Mesh::Color getRGB(float minimum, float maximum, float value)
{
	const float ratio = 2 * (value - minimum) / (maximum - minimum);
	const uint8_t r = static_cast<uint8_t>(std::max(0.f, 255 * (ratio - 1.f)));
	const uint8_t b = static_cast<uint8_t>(std::max(0.f, 255 * (1.f - ratio)));
	const uint8_t g = 255 - b - r;
	return common::Mesh::Color(r, g, b);
}

void MeshViewerWidget::calculateFace(
	common::Mesh& neutralMesh, const common::Mesh& mesh,
	const std::vector<common::Vec2i>& procrustesCorrespondences,
	const std::vector<common::Vec2i>& correspondences)
{
	std::shared_ptr<common::PointCloud> cloud =
		std::make_shared<common::PointCloud>();

	cloud->pts.reserve(mesh.n_vertices());

	size_t i = 0;
	for (common::Mesh::VertexIter vit = mesh.vertices_begin();
		 vit != mesh.vertices_end(); ++i, ++vit)
	{
		const auto point = mesh.point(*vit);
		const common::PointCloud::Point pt = {point[0], point[1], point[2]};
		cloud->pts.push_back(pt);
	}

	std::shared_ptr<common::kdTree_t> index =
		std::make_shared<common::kdTree_t>(
			3, *cloud.get(), nanoflann::KDTreeSingleIndexAdaptorParams(10));
	index->buildIndex();

	common::Target target;
	target.kdTree = index;
	target.pc = cloud;
	target.mesh = mesh;

	const bool procrustesResult = matching::sparse::alignSparse(
		neutralMesh, mesh, procrustesCorrespondences);

	this->clearMeshes();
	if (procrustesResult)
	{
#ifdef VISUALIZE_PROCRUSTES_MESH
		w.setMesh(neutralMesh);
#endif
		// disable corresponseces for a while. Maybe situation will change after
		// optimization
		nricp_->findDeformation(neutralMesh, target, {});
		// for (common::Mesh::VertexIter vit = neutralMesh.vertices_begin();
		// 	 vit != neutralMesh.vertices_end(); ++vit)
		// {
		// 	const auto& point = neutralMesh.point(*vit);
		// 	const float queryPt[3] = {point[0], point[1], point[2]};
		// 	size_t retIndex;
		// 	float outDistSqr;
		// 	nanoflann::KNNResultSet<float> resultSet(1);
		// 	resultSet.init(&retIndex, &outDistSqr);
		// 	const bool result = target.kdTree->findNeighbors(
		// 		resultSet, &queryPt[0], nanoflann::SearchParams(10));

		// 	if (result)
		// 	{
		// 		neutralMesh.set_color(*vit,
		// 							  getRGB(0, 10e-9, std::sqrt(outDistSqr)));
		// 	}
		// }

		this->setMesh(mesh);
		this->setMesh(neutralMesh);
	}
	else
	{
		spdlog::get("error")->error("Procrustes failed");
		return;
	}
}

//=============================================================================
