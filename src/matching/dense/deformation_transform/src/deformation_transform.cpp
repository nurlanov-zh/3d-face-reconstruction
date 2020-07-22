#include "deformation_transform/deformation_transform.h"
#include <memory>

namespace matching
{
namespace transform
{
DeformationTransform::DeformationTransform(
	const DeformationTransformParams& params, const common::Mesh& dst)
	: params_(params)
{
	buildKDTree(dst);
}

void DeformationTransform::applyTransform(
	const common::Mesh& src, common::Mesh& dst,
	const std::vector<common::Vec2i>& correspondences)
{
	const auto closestCorr = getCorrespondences(src, dst);

	transform(src, dst, params_.wSmoothness, params_.wIdentity, 0);

	for (int i = 0; i < 4; ++i)
	{
		transform(src, dst, params_.wSmoothness, params_.wIdentity, params_.wClosestMin + i * (params_.wClosestMax - params_.wClosestMin) / 4);
	}
}

std::vector<common::Vec2i> DeformationTransform::getCorrespondences(const common::Mesh& src, const common::Mesh& dst)
{

}

void DeformationTransform::buildKDTree(const common::Mesh& mesh)
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

	kdTree_ =
		std::make_shared<common::kdTree_t>(
			3, *cloud.get(), nanoflann::KDTreeSingleIndexAdaptorParams(10));
	kdTree_->buildIndex();
}
}  // namespace transform
}  // namespace matching