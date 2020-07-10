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
#include <data_reader/data_reader.h>
#include <landmark_detection/face_landmark_detection.h>
#include <non_rigid_icp/non_rigid_icp.h>
#include <sparse/sparse_aligner.h>

#include <QApplication>
#include <QFileDialog>
#include <QMainWindow>
#include <QMenuBar>
#include <QMessageBox>
#include <fstream>
#include <iostream>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "MeshViewerWidget.hh"

const std::string DETECTION_MODEL_PATH =
	"../data/detector/shape_predictor_68_face_landmarks.dat";
void create_menu(QMainWindow& w);
void usage_and_exit(int xcode);
void calculateFace(common::Mesh& neutralMesh, const common::Mesh& mesh,
				   matching::refinement::NRICP& nricp,
				   const std::vector<common::Vec2i>& correspondences,
				   MeshViewerWidget& w);

int main(int argc, char** argv)
{
	spdlog::set_level(spdlog::level::from_str("debug"));

	spdlog::stdout_color_mt("console");
	spdlog::stderr_color_mt("stderr");

	// OpenGL check
	QApplication::setColorSpec(QApplication::CustomColor);
	QApplication app(argc, argv);

	if (!QGLFormat::hasOpenGL())
	{
		QString msg = "System has no OpenGL support!";
		QMessageBox::critical(0, QString("OpenGL"), msg + QString(argv[1]));
		return -1;
	}

	bool useRGBDSequence = true;
	int c;
	while ((c = getopt(argc, argv, "j")) != -1)
	{
		switch (c)
		{
			case 'j':
				useRGBDSequence = false;
				break;
		}
	}

	OpenMesh::IO::Options opt;
	// enable most options for now
	opt += OpenMesh::IO::Options::VertexColor;
	opt += OpenMesh::IO::Options::VertexNormal;
	opt += OpenMesh::IO::Options::VertexTexCoord;
	opt += OpenMesh::IO::Options::FaceColor;
	opt += OpenMesh::IO::Options::FaceNormal;
	opt += OpenMesh::IO::Options::FaceTexCoord;

	// read data
	utils::DataReader dataReader("../data", opt, 0);

	// create widget
	QMainWindow mainWin;
	MeshViewerWidget w(&mainWin);
	w.setOptions(opt);
	mainWin.setCentralWidget(&w);

	create_menu(mainWin);

	mainWin.resize(1280, 720);
	mainWin.show();

	common::Mesh& neutralMesh = dataReader.getNeutralMesh();
	if (!neutralMesh.has_vertex_normals())
	{
		neutralMesh.request_vertex_normals();
	}

	matching::refinement::NRICPParams params;
	params.numOfEdges = neutralMesh.n_edges();
	params.numOfVertices = neutralMesh.n_vertices();
	params.numOfLandmarks = 68;
	params.betaInit = 1;
	params.alphaInit = 50;
	params.alphaMin = 1;
	params.numOfOuterIterations = 3;
	auto nricp = matching::refinement::NRICP(params);

	if (useRGBDSequence)
	{
		landmark_detection::LandmarkDetection lmd(DETECTION_MODEL_PATH);
		const auto& assignedLandmarks = dataReader.getAssignedLandmarks();
		while (dataReader.isNextRGBDExists())
		{
			const auto nextRgbd = dataReader.nextRGBD();
			if (!nextRgbd.has_value())
			{
				spdlog::get("stderr")->error("No rgbd sequence! Exit");
				return -1;
			}

			const auto& landmarks = lmd.detect(nextRgbd.value().first);

			std::vector<common::Vec2i> correspondences;

			common::Mesh mesh;
			mesh.request_face_normals();
			mesh.request_face_colors();
			mesh.request_vertex_normals();
			mesh.request_vertex_colors();
			mesh.request_vertex_texcoords2D();

			// dirty but who cares
			for (size_t i = 0; i < landmarks.size(); ++i)
			{
				const auto lm3d = nextRgbd.value().second->at(
					landmarks[i](0) / 2, landmarks[i](1) / 2);
				if (!std::isnan(lm3d.x) && !std::isnan(lm3d.y) &&
					!std::isnan(lm3d.z))
				{
					const common::Mesh::VertexHandle& handle = mesh.add_vertex(
						common::Mesh::Point(lm3d.x, -lm3d.y, -lm3d.z));
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
				if (it->x < 0.5 && it->x > -0.5 && it->y > -2.0 &&
					it->y < 2.0 && it->z > 1.1 && it->z < 1.5)
				{
					uint32_t rgb = *reinterpret_cast<uint32_t*>(&(it->rgb));
					uint8_t r = (rgb >> 16) & 0x0000ff;
					uint8_t g = (rgb >> 8) & 0x0000ff;
					uint8_t b = (rgb)&0x0000ff;

					const auto& handle = mesh.add_vertex(
						common::Mesh::Point(it->x, -it->y, -it->z));
					mesh.set_color(handle, {r, g, b});
				}
			}
			mesh.update_vertex_normals();
			mesh.update_face_normals();

			calculateFace(neutralMesh, mesh, nricp, correspondences, w);
		}
	}
	else
	{
		const auto& correspondences = dataReader.getCorrespondences();
		common::Mesh& kinectMesh = dataReader.getKinectMesh();
		if (!kinectMesh.has_vertex_normals())
		{
			kinectMesh.request_vertex_normals();
		}

		calculateFace(neutralMesh, kinectMesh, nricp, correspondences, w);

		kinectMesh.release_vertex_normals();
	}

	// load scene if specified on the command line
	if (++optind < argc)
	{
		w.open_texture_gui(argv[optind]);
	}

	neutralMesh.release_vertex_normals();

	return app.exec();
}

common::Mesh::Color getRGB(float minimum, float maximum, float value)
{
	const float ratio = 2 * (value - minimum) / (maximum - minimum);
	const uint8_t r = static_cast<uint8_t>(std::max(0.f, 255 * (ratio - 1.f)));
	const uint8_t b = static_cast<uint8_t>(std::max(0.f, 255 * (1.f - ratio)));
	const uint8_t g = 255 - b - r;
	return common::Mesh::Color(r, g, b);
}

void calculateFace(common::Mesh& neutralMesh, const common::Mesh& mesh,
				   matching::refinement::NRICP& nricp,
				   const std::vector<common::Vec2i>& correspondences,
				   MeshViewerWidget& w)
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

	const bool procrustesResult =
		matching::sparse::alignSparse(neutralMesh, mesh, correspondences);

	w.clearMeshes();
	if (procrustesResult)
	{
#ifdef VISUALIZE_PROCRUSTES_MESH
		w.setMesh(neutralMesh);
#endif
		// disable corresponseces for a while. Maybe situation will change after
		// optimization
		nricp.findDeformation(neutralMesh, target, {});
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
				neutralMesh.set_color(*vit, getRGB(0, 0.01, std::sqrt(outDistSqr)));
			}

		}

		w.setMesh(mesh);
		w.setMesh(neutralMesh);
	}
	else
	{
		spdlog::get("error")->error("Procrustes failed");
		return;
	}
}

void create_menu(QMainWindow& w)
{
	using namespace Qt;
	QMenu* fileMenu = w.menuBar()->addMenu(w.tr("&Control"));

	QAction* openAct = new QAction(w.tr("&Next frame..."), &w);
	openAct->setShortcut(w.tr("Ctrl+N"));
	openAct->setStatusTip(w.tr("Next frame"));
	QObject::connect(openAct, SIGNAL(triggered()), w.centralWidget(),
					 SLOT(next_frame()));
	fileMenu->addAction(openAct);

	QAction* texAct = new QAction(w.tr("&Play..."), &w);
	texAct->setShortcut(w.tr("Ctrl+P"));
	texAct->setStatusTip(w.tr("Playing"));
	QObject::connect(texAct, SIGNAL(triggered()), w.centralWidget(),
					 SLOT(play()));
	fileMenu->addAction(texAct);
}
