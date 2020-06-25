#pragma once

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/NonLinearOptimization>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

namespace common
{
using namespace OpenMesh;
using namespace OpenMesh::Attributes;
using namespace Eigen;

struct MeshViewerWidgetTraits : public OpenMesh::DefaultTraits
{
	HalfedgeAttributes(OpenMesh::Attributes::PrevHalfedge);
};

typedef OpenMesh::TriMesh_ArrayKernelT<MeshViewerWidgetTraits> Mesh;

struct float4
{
	float x;
	float y;
	float z;
	float w;
};
}  // namespace common
