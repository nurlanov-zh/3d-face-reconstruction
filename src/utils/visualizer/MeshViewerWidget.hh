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

#pragma once

//== INCLUDES =================================================================
#include "MeshViewerWidgetT.hh"

#include <common/data_types.h>
#include <data_reader/data_reader.h>
#include <face_model/face_model.h>
#include <landmark_detection/face_landmark_detection.h>
#include <non_rigid_icp/non_rigid_icp.h>
#include <sparse/sparse_aligner.h>

#include <sophus/se3.hpp>

#include <OpenMesh/Tools/Utils/getopt.h>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Tools/Utils/Timer.hh>
#include <QFileDialog>
#include <QMessageBox>
#include <QString>
#include <QWidget>

#include <iostream>
#include <memory>

//== CLASS DEFINITION =========================================================

using namespace OpenMesh;
using namespace OpenMesh::Attributes;

//== CLASS DEFINITION =========================================================

class MeshViewerWidget : public MeshViewerWidgetT<common::Mesh>
{
	Q_OBJECT

   public:
	/// default constructor
	explicit MeshViewerWidget(bool sequence, const OpenMesh::IO::Options& opt,
							  QWidget* parent = 0);

	OpenMesh::IO::Options& options() { return _options; }
	const OpenMesh::IO::Options& options() const { return _options; }
	void setOptions(const OpenMesh::IO::Options& opts) { _options = opts; }

   public slots:
	void play();

	void next_frame();

   private:
	void calculateFace(
		common::Mesh& neutralMesh, const common::Mesh& mesh,
		const common::Mesh& meshVis,
		const std::vector<common::Vec2i>& procrustesCorrespondences,
		const std::vector<common::Vec2i>& correspondences);

	common::Vec2i transformImageToDepth(const common::Vec2i& uv);

	void saveImage(const std::string& filename);

   private:
	bool seq_;
	std::unique_ptr<utils::DataReader> dataReader_;
	std::unique_ptr<matching::refinement::NRICP> nricp_;
	std::unique_ptr<matching::optimize::FaceModel> faceModel_;
	OpenMesh::IO::Options _options;
};
