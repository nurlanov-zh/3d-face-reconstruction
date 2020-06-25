#include <sparse/procrustes.h>
#include <sparse/sparse_aligner.h>
#include <spdlog\spdlog.h>
#include <fstream>
#include <iostream>

bool SparseAligner::alignSparse(common::Mesh& sourceMesh,
							   common::Mesh& targetMesh,
							   std::vector<common::Vec2f> correspondences)
{
	std::vector<common::Vector3f> sourcePoints;
	std::vector<common::Vector3f> targetPoints;
	for (int i = 0; i < correspondences.size(); i++)
	{
		common::Mesh::VertexIter v_it = sourceMesh.vertices_begin();
		while (v_it.handle().idx() < correspondences[i][0] &&
			   v_it != sourceMesh.vertices_end())
		{
			v_it++;
		}
		if (v_it.handle().idx() != correspondences[i][0])
		{
			spdlog::get("stderr")->error(
				"Wrong source correspondences ids! Abort!");
			throw std::runtime_error("Correspondences in Source");
		}
		sourcePoints.push_back(common::Vector3f(sourceMesh.point(*v_it)[0],
												sourceMesh.point(*v_it)[1],
												sourceMesh.point(*v_it)[2]));

		v_it = targetMesh.vertices_begin();
		while (v_it.handle().idx() < correspondences[i][1] &&
			   v_it != targetMesh.vertices_end())
		{
			v_it++;
		}
		if (v_it.handle().idx() != correspondences[i][1])
		{
			spdlog::get("stderr")->error(
				"Wrong target correspondences ids! Abort!");
			throw std::runtime_error("Correspondences in Target");
		}
		targetPoints.push_back(common::Vector3f(targetMesh.point(*v_it)[0],
												targetMesh.point(*v_it)[1],
												targetMesh.point(*v_it)[2]));
	}

	ProcrustesAligner aligner;
	common::Matrix4f estimatedPose =
		aligner.estimatePose(sourcePoints, targetPoints);

	
	std::ofstream outFile("../data/procrustes.off");
	if (!outFile.is_open())
	{
		spdlog::get("stderr")->warn("Procrustes file couldn't be loaded!");
		return false;
	}
	outFile << "COFF" << std::endl;
	// TODO- WRITE FACES
	outFile << sourceMesh.vertices_end().handle().idx() << " "
			<< "0"
			<< " 0" << std::endl;

	common::Mesh::VertexIter v_it = sourceMesh.vertices_sbegin();
	while (v_it != sourceMesh.vertices_end())
	{
		common::Vec3f point = sourceMesh.point(v_it.handle());
		common::Vector4f newPoint =
			estimatedPose * common::Vector4f(point[0], point[1], point[2], 1);
		outFile << newPoint(0) << " " << newPoint(1) << " " << newPoint(2)
				<< " " << 255 << " " << 0 << " " << 0 << " " << 0 << std::endl;
		v_it++;
	}
	outFile.close();
	return true;
}