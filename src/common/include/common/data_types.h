#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/StdVector>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <unsupported/Eigen/NonLinearOptimization>

#include <memory>

#include "nanoflann.hpp"

namespace common
{
using namespace OpenMesh;
using namespace OpenMesh::Attributes;
using namespace Eigen;

struct MeshViewerWidgetTraits : public OpenMesh::DefaultTraits
{
	HalfedgeAttributes(OpenMesh::Attributes::PrevHalfedge);

	FaceAttributes(OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color);

	VertexAttributes(OpenMesh::Attributes::Normal |
					 OpenMesh::Attributes::Color);
};

typedef OpenMesh::TriMesh_ArrayKernelT<MeshViewerWidgetTraits> Mesh;

struct float4
{
	float x;
	float y;
	float z;
	float w;
};

struct PointCloud
{
	struct Point
	{
		float x;
		float y;
		float z;
	};

	std::vector<Point> pts;

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return pts.size(); }

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate
	// value, the
	//  "if/else's" are actually solved at compile time.
	inline float kdtree_get_pt(const size_t idx, const size_t dim) const
	{
		if (dim == 0)
			return pts[idx].x;
		else if (dim == 1)
			return pts[idx].y;
		else
			return pts[idx].z;
	}

	// Optional bounding-box computation: return false to default to a standard
	// bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned
	//   in "bb" so it can be avoided to redo it again. Look at bb.size() to
	//   find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const
	{
		return false;
	}
};

typedef nanoflann::KDTreeSingleIndexAdaptor<
	nanoflann::L2_Simple_Adaptor<float, PointCloud>, PointCloud, 3>
	kdTree_t;

struct Target
{
	std::shared_ptr<kdTree_t> kdTree;
	std::shared_ptr<PointCloud> pc;
	Mesh mesh;
};

}  // namespace common
