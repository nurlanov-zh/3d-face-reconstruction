#include <sparse/procrustes.h>
#include <sparse/sparse_aligner.h>
#include <spdlog\spdlog.h>
#include <fstream>

const std::string PROCRUSTES_MESH_LOCATION = "../data/procrustes.off";
const std::string SOURCE_MESH_LOCATION = "../data/kinectdata.off";

std::ifstream& GotoLine(std::ifstream& file, unsigned int n)
{
	file.seekg(std::ios::beg);
	for (int i = 0; i < n - 1; ++i)
	{
		file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}
	return file;
}

bool matching::sparse::alignSparse(common::Mesh& sourceMesh,
								   common::Mesh& targetMesh,
								   std::vector<common::Vec2f> correspondences)
{
	std::vector<common::Vector3f> sourcePoints;
	std::vector<common::Vector3f> targetPoints;

	common::Mesh::VertexIter v_it = sourceMesh.vertices_begin();
	for (int i = 0; i < correspondences.size(); i++)
	{
		v_it = sourceMesh.vertices_begin();

		// Fill source correspondences
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

		// Fill target correspondences
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

	common::Matrix4f estimatedPose = estimatePose(sourcePoints, targetPoints);

	if (!transformAndWrite(sourceMesh, estimatedPose))
	{
		spdlog::get("stderr")->warn(
			"Could not write procrustes transformation!");
		return false;
	}

	return true;
}

bool matching::sparse::transformAndWrite(common::Mesh& sourceMesh,
										 common::Matrix4f estimatedPose)
{
	std::ofstream outFile(PROCRUSTES_MESH_LOCATION);
	std::ifstream orgMesh(SOURCE_MESH_LOCATION);

	if (!outFile.is_open())
	{
		spdlog::get("stderr")->warn("Procrustes file couldn't be loaded!");
		return false;
	}

	// Mesh file header
	outFile << "COFF" << std::endl;
	GotoLine(orgMesh, 2);
	std::string line;
	std::getline(orgMesh, line);
	outFile << line << std::endl;

	// Write transformed points
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

	// Copy triangles from original mesh
	GotoLine(orgMesh, v_it.handle().idx() + 3);
	std::string point;
	while (orgMesh.eof() == 0)
	{
		std::getline(orgMesh, point);
		outFile << point << std::endl;
	}

	orgMesh.close();
	outFile.close();

	return true;
}