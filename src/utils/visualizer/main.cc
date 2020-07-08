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

void create_menu(QMainWindow &w);
void usage_and_exit(int xcode);

int main(int argc, char **argv)
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

	OpenMesh::IO::Options opt;
	// enable most options for now
	opt += OpenMesh::IO::Options::VertexColor;
	opt += OpenMesh::IO::Options::VertexNormal;
	opt += OpenMesh::IO::Options::VertexTexCoord;
	opt += OpenMesh::IO::Options::FaceColor;
	opt += OpenMesh::IO::Options::FaceNormal;
	opt += OpenMesh::IO::Options::FaceTexCoord;

	// read data
	utils::DataReader dataReader("../data", opt);

	// create widget
	QMainWindow mainWin;
	MeshViewerWidget w(&mainWin);
	w.setOptions(opt);
	mainWin.setCentralWidget(&w);

	create_menu(mainWin);

	mainWin.resize(1280, 720);
	mainWin.show();

	common::Mesh &kinectMesh = dataReader.getKinectMesh();
	common::Mesh &neutralMesh = dataReader.getNeutralMesh();
	const auto &correspondences = dataReader.getCorrespondences();

	if (!kinectMesh.has_vertex_normals())
	{
		kinectMesh.request_vertex_normals();
	}

	if (!neutralMesh.has_vertex_normals())
	{
		neutralMesh.request_vertex_normals();
	}

	std::shared_ptr<common::PointCloud> cloud =
		std::make_shared<common::PointCloud>();
	cloud->pts.reserve(kinectMesh.n_vertices());
	size_t i = 0;
	for (common::Mesh::VertexIter vit = kinectMesh.vertices_begin();
		 vit != kinectMesh.vertices_end(); ++i, ++vit)
	{
		const auto point = kinectMesh.point(*vit);
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
	target.mesh = kinectMesh;

	const bool procrustesResult =
		matching::sparse::alignSparse(neutralMesh, kinectMesh, correspondences);

	matching::refinement::NRICPParams params;
	params.numOfEdges = neutralMesh.n_edges();
	params.numOfVertices = neutralMesh.n_vertices();
	params.numOfLandmarks = correspondences.size();
	params.betaInit = 1;
	params.alphaInit = 100;
	params.alphaMin = 50;
	params.numOfOuterIterations = 3;
	auto nricp = matching::refinement::NRICP(params);

	if (procrustesResult)
	{
#ifdef VISUALIZE_PROCRUSTES_MESH
		w.setMesh(neutralMesh);
#endif
		nricp.findDeformation(neutralMesh, target, {});
		w.setMesh(neutralMesh);
	}
	else
	{
		spdlog::get("error")->error("Procrustes failed");
		return -1;
	}

	w.setMesh(kinectMesh);

	// load scene if specified on the command line
	if (++optind < argc)
	{
		w.open_texture_gui(argv[optind]);
	}

	kinectMesh.release_vertex_normals();
	neutralMesh.release_vertex_normals();

	return app.exec();
}

void create_menu(QMainWindow &w)
{
	using namespace Qt;
	QMenu *fileMenu = w.menuBar()->addMenu(w.tr("&Control"));

	QAction *openAct = new QAction(w.tr("&Next frame..."), &w);
	openAct->setShortcut(w.tr("Ctrl+N"));
	openAct->setStatusTip(w.tr("Next frame"));
	QObject::connect(openAct, SIGNAL(triggered()), w.centralWidget(),
					 SLOT(next_frame()));
	fileMenu->addAction(openAct);

	QAction *texAct = new QAction(w.tr("&Play..."), &w);
	texAct->setShortcut(w.tr("Ctrl+P"));
	texAct->setStatusTip(w.tr("Playing"));
	QObject::connect(texAct, SIGNAL(triggered()), w.centralWidget(),
					 SLOT(play()));
	fileMenu->addAction(texAct);
}

void usage_and_exit(int xcode)
{
	std::cout << "Usage: meshviewer [-s] [mesh] [texture]\n" << std::endl;
	std::cout << "Options:\n"
			  << "  -b\n"
			  << "    Assume input to be binary.\n\n"
			  << "  -s\n"
			  << "    Reverse byte order, when reading binary files.\n"
			  << std::endl;
	exit(xcode);
}
