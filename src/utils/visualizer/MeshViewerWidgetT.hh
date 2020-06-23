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
#include "QGLViewerWidget.hh"

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/IO/Options.hh>
#include <OpenMesh/Core/Mesh/Attributes.hh>
#include <OpenMesh/Core/Utils/GenProg.hh>
#include <OpenMesh/Core/Utils/color_cast.hh>
#include <OpenMesh/Tools/Utils/StripifierT.hh>
#include <OpenMesh/Tools/Utils/Timer.hh>

#include <string>

//== FORWARDS =================================================================

class QImage;

//== CLASS DEFINITION =========================================================

template <typename M>
class MeshViewerWidgetT : public QGLViewerWidget
{
   public:
	typedef M Mesh;
	typedef OpenMesh::StripifierT<Mesh> MyStripifier;

   public:
	/// default constructor
	explicit MeshViewerWidgetT(QWidget* _parent = 0)
		: QGLViewerWidget(_parent),
		  tex_id_(0),
		  tex_mode_(GL_MODULATE),
		  use_color_(true),
		  show_vnormals_(false),
		  show_fnormals_(false),
		  normal_scale_(1.0)
	{
		add_draw_mode("Points");
		add_draw_mode("Hidden-Line");
#if defined(OM_USE_OSG) && OM_USE_OSG
		add_draw_mode("OpenSG Indices");
#endif
	}

	/// destructor
	~MeshViewerWidgetT() {}

   public:
	/// load texture
	virtual bool open_texture(const char* _filename);
	bool set_texture(QImage& _texsrc);

	void setMesh(const Mesh& mesh);

   protected:
	/// inherited drawing method
	virtual void setupMesh();
	virtual void draw_scene(const std::string& _draw_mode) override;

   protected:
	/// draw the mesh
	virtual void draw_openmesh(const std::string& _drawmode);

	void glVertex(const typename Mesh::VertexHandle _vh, const Mesh& mesh)
	{
		glVertex3fv(&mesh.point(_vh)[0]);
	}

	void glVertex(const typename Mesh::Point& _p) { glVertex3fv(&_p[0]); }

	void glNormal(const typename Mesh::VertexHandle _vh, const Mesh& mesh)
	{
		glNormal3fv(&mesh.normal(_vh)[0]);
	}

	void glTexCoord(const typename Mesh::VertexHandle _vh, const Mesh& mesh)
	{
		glTexCoord2fv(&mesh.texcoord(_vh)[0]);
	}

	void glColor(const typename Mesh::VertexHandle _vh, const Mesh& mesh)
	{
		glColor3ubv(&mesh.color(_vh)[0]);
	}

	// face properties

	void glNormal(const typename Mesh::FaceHandle _fh, const Mesh& mesh)
	{
		glNormal3fv(&mesh.normal(_fh)[0]);
	}

	void glColor(const typename Mesh::FaceHandle _fh, const Mesh& mesh)
	{
		glColor3ubv(&mesh.color(_fh)[0]);
	}

	void glMaterial(const typename Mesh::FaceHandle _fh, const Mesh& mesh,
					int _f = GL_FRONT_AND_BACK, int _m = GL_DIFFUSE)
	{
		OpenMesh::Vec3f c =
			OpenMesh::color_cast<OpenMesh::Vec3f>(mesh.color(_fh));
		OpenMesh::Vec4f m(c[0], c[1], c[2], 1.0f);

		glMaterialfv(_f, _m, &m[0]);
	}

   protected:  // Strip support

   protected:  // inherited
	virtual void keyPressEvent(QKeyEvent* _event) override;

   protected:
	std::shared_ptr<spdlog::logger> consoleLog_;
	std::shared_ptr<spdlog::logger> errLog_;

	GLuint tex_id_;
	GLint tex_mode_;
	OpenMesh::IO::Options opt_;  // mesh file contained texcoords?

	std::vector<Mesh> meshes_;

	bool use_color_;
	bool show_vnormals_;
	bool show_fnormals_;
	float normal_scale_;
	OpenMesh::FPropHandleT<typename Mesh::Point> fp_normal_base_;
};

//=============================================================================
#if defined(OM_INCLUDE_TEMPLATES)
#define OPENMESH_MESHVIEWERWIDGET_TEMPLATES
#include "MeshViewerWidgetT_impl.hh"
#endif
//=============================================================================
