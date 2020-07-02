#include "icp/icp.h"
#include <spdlog/spdlog.h>
#include <iostream>
#include "sparse/procrustes.h"

using namespace matching::sparse;
namespace tracking
{
namespace icp
{
bool tracking::icp::trackICP(common::Mesh& sourceMesh, common::Mesh& targetMesh,
							 const std::vector<common::Vec2i>& correspondences)
{
	common::Vector3f mean = corrMean(correspondences, targetMesh);
	removeOutliers(targetMesh, mean);

	std::vector<common::Vector3f> sourcePoints;
	std::vector<common::Vector3f> targetPoints;

	for (common::Mesh::VertexIter vit = sourceMesh.vertices_begin();
		 vit != sourceMesh.vertices_end(); ++vit)
	{
		auto point = sourceMesh.point(*vit);
		sourcePoints.push_back(common::Vector3f(point[0], point[1], point[2]));
	}
	for (common::Mesh::VertexIter vit = targetMesh.vertices_begin();
		 vit != targetMesh.vertices_end(); ++vit)
	{
		auto point = targetMesh.point(*vit);
		targetPoints.push_back(common::Vector3f(point[0], point[1], point[2]));
	}

	common::Matrix4f estimatedPose = estimatePose(sourcePoints, targetPoints);

	// Print estimatedPose in Debug
	std::stringstream ss;
	ss << estimatedPose;
	spdlog::get("console")->debug("ICP Result: " + ss.str());

	if (!transformAndWrite(sourceMesh, estimatedPose))
	{
		spdlog::get("stderr")->warn(
			"Could not write procrustes transformation!");
		throw std::runtime_error("Procrustes mesh");
	}

	return true;
}

common::Vector3f corrMean(const std::vector<common::Vec2i>& correspondences,
						  const common::Mesh targetMesh)
{
	common::Vector3f mean = common::Vector3f::Zero();
	for (int i = 0; i < correspondences.size(); i++)
	{
		const auto targetPoint =
			targetMesh.point(OpenMesh::VertexHandle(correspondences[i][1]));
		mean +=
			common::Vector3f(targetPoint[0], targetPoint[1], targetPoint[2]);
	}
	mean = (1.0f / correspondences.size()) * mean;
	return mean;
}

void removeOutliers(common::Mesh& mesh, common::Vector3f mean)
{
	bool keep = false;
	for (common::Mesh::VertexIter vit = mesh.vertices_begin();
		 vit != mesh.vertices_end(); ++vit)
	{
		keep = true;
		auto point = mesh.point(vit.handle());
		common::Vector3f vecPoint =
			common::Vector3f(point[0], point[1], point[2]);
		float distance = (mean - vecPoint).norm();
		if (distance > 0.15f) keep = false;

		if (!keep)
		{
			mesh.request_vertex_status();
			mesh.request_edge_status();
			mesh.request_halfedge_status();
			mesh.request_face_status();
			mesh.request_vertex_status();

			mesh.request_vertex_status();
			mesh.delete_vertex(vit.handle(), true);
		}
	}
	mesh.garbage_collection();
}

common::Matrix4f tracking::icp::estimatePose(
	const std::vector<common::Vector3f> sourceMesh,
	const std::vector<common::Vector3f> targetMesh)
{
	std::unique_ptr<NearestNeighborSearch> m_nearestNeighborSearch =
		std::make_unique<NearestNeighborSearchFlann>();
	m_nearestNeighborSearch->setMatchingMaxDistance(tracking::icp::maxMatchingDist);
	m_nearestNeighborSearch->buildIndex(targetMesh);

	// The initial estimate can be given as an argument.
	common::Matrix4f estimatedPose = common::Matrix4f::Identity();

	for (int i = 0; i < tracking::icp::nIterations; ++i)
	{
		// Compute the matches.
		std::cout << i << ": Matching points ..." << std::endl;
		clock_t begin = clock();

		auto transformedPoints = transformPoints(sourceMesh, estimatedPose);

		auto matches = m_nearestNeighborSearch->queryMatches(transformedPoints);
		// pruneCorrespondences(transformedNormals, target.getNormals(),
		// matches);

		clock_t end = clock();
		double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
		std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

		std::vector<common::Vector3f> sourcePoints;
		std::vector<common::Vector3f> targetPoints;

		for (int j = 0; j < transformedPoints.size(); j++)
		{
			const auto& match = matches[j];
			if (match.idx >= 0)
			{
				sourcePoints.push_back(transformedPoints[j]);
				targetPoints.push_back(targetMesh[match.idx]);
			}
		}
		estimatedPose = estimatePosePointToPoint(sourcePoints, targetPoints) *
						estimatedPose;

		std::cout << "Optimization iteration done." << std::endl;
	}

	return estimatedPose;
}

std::vector<common::Vector3f> tracking::icp::transformPoints(
	const std::vector<common::Vector3f>& sourcePoints,
	const common::Matrix4f& pose)
{
	std::vector<common::Vector3f> transformedPoints;
	transformedPoints.reserve(sourcePoints.size());

	const auto rotation = pose.block(0, 0, 3, 3);
	const auto translation = pose.block(0, 3, 3, 1);

	for (const auto& point : sourcePoints)
	{
		transformedPoints.push_back(rotation * point + translation);
	}

	return transformedPoints;
}

common::Matrix4f tracking::icp::estimatePosePointToPoint(
	const std::vector<common::Vector3f>& sourcePoints,
	const std::vector<common::Vector3f>& targetPoints)
{
	common::Matrix4f estimatedPose =
		matching::sparse::estimatePose(sourcePoints, targetPoints);

	return estimatedPose;
}

bool transformAndWrite(common::Mesh& sourceMesh,
					   const common::Matrix4f& estimatedPose)
{
	// Write transformed points
	common::Mesh::VertexIter v_it = sourceMesh.vertices_sbegin();
	while (v_it != sourceMesh.vertices_end())
	{
		const common::Vec3f point = sourceMesh.point(*v_it);
		const common::Vector4f newPoint =
			estimatedPose * common::Vector4f(point[0], point[1], point[2], 1);

		sourceMesh.set_point(*v_it, {newPoint(0), newPoint(1), newPoint(2)});

		v_it++;
	}

	return true;
}

}  // namespace icp
}  // namespace tracking
