#pragma once

#include <common/data_types.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/IO/Options.hh>

namespace matching
{
namespace sparse
{
bool alignSparse(common::Mesh& sourceMesh, common::Mesh& targetMesh,
				 std::vector<common::Vec2f> correspondences);

bool transformAndWrite(common::Mesh& sourceMesh,
					   common::Matrix4f estimatedPose);

}  // namespace sparse
}  // namespace matching