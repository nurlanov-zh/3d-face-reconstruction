#pragma once

#include <common/data_types.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/IO/Options.hh>

#define EXPORT_PROCRUSTES_MESH 0;

namespace matching
{
namespace sparse
{

common::Matrix4f alignSparse(common::Mesh& sourceMesh, common::Mesh& targetMesh,
							 std::vector<common::Vec2i> correspondences);

bool transformAndWrite(common::Mesh& sourceMesh,
					   common::Matrix4f estimatedPose);

}  // namespace sparse
}  // namespace matching