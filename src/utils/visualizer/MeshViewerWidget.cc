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
#include "face_model/face_model.h"

//== IMPLEMENTATION ==========================================================

/// default constructor

const std::string DETECTION_MODEL_PATH =
	"../data/detector/shape_predictor_68_face_landmarks.dat";

constexpr int32_t WIDTH = 1280;
constexpr int32_t BAR = 100;
constexpr int32_t HEIGHT = 720;
const std::unordered_set<int32_t> landmarksIdsProcrustes = {
	8,  17, 19, 21, 22, 24, 26, 27, 30, 31, 33, 35, 36, 37, 38, 39, 40, 41,
	42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59};

MeshViewerWidget::MeshViewerWidget(bool sequence,
								   const OpenMesh::IO::Options& opt,
								   QWidget* parent)
	: MeshViewerWidgetT<common::Mesh>(parent), seq_(sequence)
{
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
	params.alphaInit = 10000;
	params.alphaMin = 10;
	params.numOfOuterIterations = 100;
	nricp_.reset(new matching::refinement::NRICP(params));

	matching::optimize::FaceModelParams faceModelParams;

	faceModel_.reset(new matching::optimize::FaceModel(faceModelParams));
	faceModel_->setShapeBasis(dataReader_->getShapeBasis());
	faceModel_->setShapeBasisDev(dataReader_->getShapeBasisDev());
	faceModel_->setExpressionsBasis(dataReader_->getExpressionsBasis());
	faceModel_->setExpressionsBasisDev(dataReader_->getExpressionsBasisDev());

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

			cv::Mat image = nextRgbd.value().first;
			const auto& landmarks = lmd.detect(image);

			std::vector<common::Vec2i> correspondences;
			std::vector<common::Vec2i> procrustesCorrespondences;

			common::Mesh meshVis;
			meshVis.request_face_normals();
			meshVis.request_face_colors();
			meshVis.request_vertex_normals();
			meshVis.request_vertex_colors();
			meshVis.request_vertex_texcoords2D();

			common::Mesh mesh;
			mesh.request_face_normals();
			mesh.request_face_colors();
			mesh.request_vertex_normals();
			mesh.request_vertex_colors();
			mesh.request_vertex_texcoords2D();

			// dirty but who cares
			std::vector<std::pair<int32_t, pcl::PointXYZRGB>> points;
			for (size_t i = 0; i < landmarks.size(); ++i)
			{
				if (landmarksIdsProcrustes.find(i) !=
					landmarksIdsProcrustes.end())
				{
					cv::line(image,
							 {static_cast<int32_t>(landmarks[i](0)) + 5,
							  static_cast<int32_t>(landmarks[i](1)) - 5},
							 {static_cast<int32_t>(landmarks[i](0)) - 5,
							  static_cast<int32_t>(landmarks[i](1)) + 5},
							 {0, 0, 255}, 3);
					cv::line(image,
							 {static_cast<int32_t>(landmarks[i](0)) - 5,
							  static_cast<int32_t>(landmarks[i](1)) - 5},
							 {static_cast<int32_t>(landmarks[i](0)) + 5,
							  static_cast<int32_t>(landmarks[i](1)) + 5},
							 {0, 0, 255}, 3);
				}

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

			for (size_t i = 0; i < points.size(); i++)
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

					const common::Mesh::VertexHandle& handleVis =
						meshVis.add_vertex(common::Mesh::Point(
							points[i].second.x, -points[i].second.y,
							-points[i].second.z));

					if (landmarksIdsProcrustes.find(points[i].first) !=
						landmarksIdsProcrustes.end())
					{
						procrustesCorrespondences.push_back(
							{assignedLandmarks[i], handle.idx()});
						mesh.set_color(handle, {255, 0, 0});
						meshVis.set_color(handleVis, {255, 0, 0});
						neutralMesh.set_color(
							common::Mesh::VertexHandle(assignedLandmarks[i]),
							{255, 0, 0});
					}
					correspondences.push_back(
						{assignedLandmarks[i], handle.idx()});
				}
			}

			size_t i = 0;
			cv::Mat idxs(nextRgbd.value().second->height,
						 nextRgbd.value().second->width, CV_32S, -1);
			for (size_t y = 0; y < nextRgbd.value().second->height; ++y)
			{
				for (size_t x = 0; x < nextRgbd.value().second->width; ++x)
				{
					const auto point = nextRgbd.value().second->at(x, y);
					if (!std::isnan(point.x) && !std::isnan(point.y) &&
						!std::isnan(point.z) && point.z < 0.6 && point.z > 0.01)
					{
						uint32_t rgb =
							*reinterpret_cast<const uint32_t*>(&(point.rgb));
						uint8_t r = (rgb >> 16) & 0x0000ff;
						uint8_t g = (rgb >> 8) & 0x0000ff;
						uint8_t b = (rgb)&0x0000ff;

						if (x % 3 == 0 && y % 3 == 0)
						{
							const auto& handle =
								mesh.add_vertex(common::Mesh::Point(
									point.x, -point.y, -point.z));
							mesh.set_color(handle, {b, g, r});
						}
						const auto& handleVis = meshVis.add_vertex(
							common::Mesh::Point(point.x, -point.y, -point.z));
						idxs.at<int32_t>(y, x) = handleVis.idx();
						meshVis.set_color(handleVis, {b, g, r});
					}
				}
			}

			const auto isSameFacet = [](const pcl::PointXYZRGB& v1,
										const pcl::PointXYZRGB& v2) -> bool {
				const Eigen::Vector3f vector(v2.x - v1.x, v2.y - v1.y,
											 v2.z - v1.z);
				const float dist =
					std::sqrt((vector * vector.transpose()).sum());
				if (std::isnan(dist))
				{
					return false;
				}

				return dist < 0.1;
			};

			const auto generateFacet =
				[&meshVis, &isSameFacet, &idxs, &nextRgbd](
					const cv::Point2i& currentIndex,
					const cv::Point2i& neighbor1,
					const cv::Point2i& neighbor2) -> void {
				const auto currentPoint =
					nextRgbd.value().second->at(currentIndex.x, currentIndex.y);
				const auto currIdx =
					idxs.at<int32_t>(currentIndex.y, currentIndex.x);
				const auto neigh1Idx =
					idxs.at<int32_t>(neighbor1.y, neighbor1.x);
				const auto neigh2Idx =
					idxs.at<int32_t>(neighbor2.y, neighbor2.x);

				if (currIdx != -1 && neigh1Idx != -1 && neigh2Idx != -1 &&
					neighbor1.y <
						static_cast<int32_t>(nextRgbd.value().second->height) &&
					neighbor1.y >= 0 &&
					neighbor2.y <
						static_cast<int32_t>(nextRgbd.value().second->height) &&
					neighbor2.y >= 0 &&
					neighbor1.x <
						static_cast<int32_t>(nextRgbd.value().second->width) &&
					neighbor1.x >= 0 &&
					neighbor2.x <
						static_cast<int32_t>(nextRgbd.value().second->width) &&
					neighbor2.x >= 0)
				{
					const auto neighbor1Point =
						nextRgbd.value().second->at(neighbor1.x, neighbor1.y);
					const auto neighbor2Point =
						nextRgbd.value().second->at(neighbor2.x, neighbor2.y);

					if (isSameFacet(currentPoint, neighbor1Point) &&
						isSameFacet(currentPoint, neighbor2Point) &&
						isSameFacet(neighbor1Point, neighbor2Point))
					{
						std::vector<common::Mesh::VertexHandle> faceVhandles;
						faceVhandles.clear();
						faceVhandles.push_back(
							common::Mesh::VertexHandle(currIdx));
						faceVhandles.push_back(
							common::Mesh::VertexHandle(neigh1Idx));
						faceVhandles.push_back(
							common::Mesh::VertexHandle(neigh2Idx));
						meshVis.add_face(faceVhandles);
					}
				}
			};

			for (int y = 0; y < idxs.rows; ++y)
			{
				for (int x = 0; x < idxs.cols; ++x)
				{
					generateFacet({x, y}, {x - 1, y + 1}, {x, y + 1});
					generateFacet({x, y}, {x, y + 1}, {x + 1, y});
				}
			}

			meshVis.update_vertex_normals();
			meshVis.update_face_normals();

			calculateFace(neutralMesh, mesh, meshVis, procrustesCorrespondences,
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

		calculateFace(neutralMesh, kinectMesh, kinectMesh, correspondences,
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
	const common::Mesh& meshVis,
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
		const auto color = mesh.color(*vit);
		const common::PointCloud::Point pt = {point[0], point[1], point[2],
											  color[0], color[1], color[2]};
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

	/*const bool procrustesResult =
		matching::sparse::alignSparse(neutralMesh, mesh,
	   procrustesCorrespondences);*/

	common::Matrix4f poseInit = matching::sparse::estimatePose(
		neutralMesh, mesh, procrustesCorrespondences);

	this->clearMeshes();
	{
#ifdef VISUALIZE_PROCRUSTES_MESH
		w.setMesh(neutralMesh);
#endif
		faceModel_->optimize(neutralMesh, target, procrustesCorrespondences,
							 poseInit);

		//matching::sparse::transformAndWrite(neutralMesh, poseInit);

		nricp_->findDeformation(neutralMesh, target, {});
		for (common::Mesh::VertexIter vit = neutralMesh.vertices_begin();
			 vit != neutralMesh.vertices_end(); ++vit)
		{
			const auto& point = neutralMesh.point(*vit);
			const float queryPt[3] = {point[0], point[1], point[2]};
			size_t retIndex;
			float outDistSqr;
			nanoflann::KNNResultSet<float> resultSet(1);
			resultSet.init(&retIndex, &outDistSqr);
			const bool result = target.kdTree->findNeighbors(
				resultSet, &queryPt[0], nanoflann::SearchParams(10));

			if (result)
			{
				neutralMesh.set_color(
					*vit, /*common::Mesh::Color(target.pc->pts[retIndex].r,
											  target.pc->pts[retIndex].g,
											  target.pc->pts[retIndex].b));*/
					getRGB(0, 5e-4, std::sqrt(outDistSqr)));
			}
		}

		this->setMesh(mesh);
		this->setMesh(neutralMesh);
	}
}

//=============================================================================
