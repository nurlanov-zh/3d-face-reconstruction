#pragma once

#include <common/data_types.h>

namespace matching
{
namespace sparse
{
bool alignSparse(common::Mesh& sourceMesh, const common::Mesh& targetMesh,
				 const std::vector<common::Vec2i>& correspondences);
}  // namespace sparse
}  // namespace matching
