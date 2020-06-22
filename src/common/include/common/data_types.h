#pragma once

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

namespace common
{
using namespace OpenMesh;
using namespace OpenMesh::Attributes;

struct MeshViewerWidgetTraits : public OpenMesh::DefaultTraits
{
	HalfedgeAttributes(OpenMesh::Attributes::PrevHalfedge);
};

typedef OpenMesh::TriMesh_ArrayKernelT<MeshViewerWidgetTraits> Mesh;
}  // namespace common