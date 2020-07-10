#include "sparse/sparse_aligner.h"
#include "sparse/procrustes.h"

#include <spdlog/spdlog.h>
#include <fstream>

namespace matching
{
namespace sparse
{
#ifdef VISUALIZE_PROCRUSTES_MESH
const std::string PROCRUSTES_MESH_LOCATION = "../data/procrustes.off";
const std::string SOURCE_MESH_LOCATION = "../data/kinectdata.off";
#endif

bool transformAndWrite(common::Mesh& sourceMesh,
					   const common::Matrix4f& estimatedPose)
{
#ifdef VISUALIZE_PROCRUSTES_MESH

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
#endif

	// Write transformed points
	common::Mesh::VertexIter v_it = sourceMesh.vertices_sbegin();
	while (v_it != sourceMesh.vertices_end())
	{
		const common::Vec3f point = sourceMesh.point(*v_it);
		const common::Vector4f newPoint =
			estimatedPose * common::Vector4f(point[0], point[1], point[2], 1);

		sourceMesh.set_point(*v_it, {newPoint(0), newPoint(1), newPoint(2)});

#ifdef VISUALIZE_PROCRUSTES_MESH
		outFile << newPoint(0) << " " << newPoint(1) << " " << newPoint(2)
				<< " " << 255 << " " << 0 << " " << 0 << " " << 0 << std::endl;
#endif
		v_it++;
	}

#ifdef VISUALIZE_PROCRUSTES_MESH
	// Copy triangles from original mesh
	GotoLine(orgMesh, v_it->idx() + 3);
	std::string point;
	while (orgMesh.eof() == 0)
	{
		std::getline(orgMesh, point);
		outFile << point << std::endl;
	}

	orgMesh.close();
	outFile.close();
#endif

	return true;
}

bool alignSparse(common::Mesh& sourceMesh, const common::Mesh& targetMesh,
				 const std::vector<common::Vec2i>& correspondences)
{
	std::vector<common::Vector3f> sourcePoints;
	std::vector<common::Vector3f> targetPoints;

	for (size_t i = 0; i < correspondences.size(); i++)
	{
		const auto sourcePoint =
			sourceMesh.point(OpenMesh::VertexHandle(correspondences[i][0]));
		const auto targetPoint =
			targetMesh.point(OpenMesh::VertexHandle(correspondences[i][1]));

		sourcePoints.push_back(
			common::Vector3f(sourcePoint[0], sourcePoint[1], sourcePoint[2]));

		targetPoints.push_back(
			common::Vector3f(targetPoint[0], targetPoint[1], targetPoint[2]));
	}

	common::Matrix4f estimatedPose = estimatePose(sourcePoints, targetPoints);

	// Print estimatedPose in Debug
	std::stringstream ss;
	ss << estimatedPose;
	spdlog::get("console")->debug(ss.str());

	if (!transformAndWrite(sourceMesh, estimatedPose))
	{
		spdlog::get("stderr")->warn(
			"Could not write procrustes transformation!");
		throw std::runtime_error("Procrustes mesh");
	}

	return true;
}

#if VISUALIZE_PROCRUSTES_MESH
std::ifstream& GotoLine(std::ifstream& file, unsigned int n)
{
	file.seekg(std::ios::beg);
	for (int i = 0; i < n - 1; ++i)
	{
		file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}
	return file;
}
#endif

}  // namespace sparse
}  // namespace matching
