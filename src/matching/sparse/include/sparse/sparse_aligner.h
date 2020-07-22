#pragma once

#include <common/data_types.h>

namespace matching
{
namespace sparse
{
bool alignSparse(common::Mesh& sourceMesh, const common::Mesh& targetMesh,
				 const std::vector<common::Vec2i>& correspondences);
common::Matrix4f estimatePose(
	const common::Mesh& sourceMesh, const common::Mesh& targetMesh,
	const std::vector<common::Vec2i>& correspondences);

bool transformAndWrite(common::Mesh& sourceMesh,
					   const common::Matrix4f& estimatedPose);
}  // namespace sparse
}  // namespace matching
