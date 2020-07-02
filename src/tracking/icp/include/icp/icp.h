#pragma once

#include "common/NearestNeighbor.h"

namespace tracking
{
namespace icp
{

const int nIterations = 15;
const float maxMatchingDist = 0.000015f;

bool trackICP(common::Mesh& sourceMesh, common::Mesh& targetMesh,
			  const std::vector<common::Vec2i>& correspondences);

common::Vector3f corrMean(const std::vector<common::Vec2i>& correspondences,
						  const common::Mesh targetPoints);

void removeOutliers(common::Mesh& mesh, common::Vector3f mean);

common::Matrix4f estimatePose(const std::vector<common::Vector3f> sourceMesh,
							  const std::vector<common::Vector3f> targetMesh);

std::vector<common::Vector3f> transformPoints(
	const std::vector<common::Vector3f>& sourcePoints,
	const common::Matrix4f& pose);

common::Matrix4f estimatePosePointToPoint(
	const std::vector<common::Vector3f>& sourcePoints,
	const std::vector<common::Vector3f>& targetPoints);

bool transformAndWrite(common::Mesh& sourceMesh,
					   const common::Matrix4f& estimatedPose);

}  // namespace icp
}  // namespace tracking