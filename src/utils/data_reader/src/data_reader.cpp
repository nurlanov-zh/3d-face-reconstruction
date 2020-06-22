#include "data_reader/data_reader.h"

namespace utils
{
const std::string NEUTRAL_FACE_NAME = "/neutral_face.ply";

DataReader::DataReader(const std::string& path, OpenMesh::IO::Options opt)
	: path_(path), opt_(opt)
{
	consoleLog_ = spdlog::get("console");
	errLog_ = spdlog::get("stderr");
}

void DataReader::readPCAFace()
{
	const std::string filename = path_ + "/" + NEUTRAL_FACE_NAME;
	if (openMesh(filename))
	{
		consoleLog_->info("PCA face successfully loaded");
		return;
	}
	consoleLog_->info("PCA face is not loaded");
}

bool DataReader::openMesh(const std::string& filename)
{
	// load mesh
	// calculate normals
	// set scene center and radius
	mesh_.request_face_normals();
	mesh_.request_face_colors();
	mesh_.request_vertex_normals();
	mesh_.request_vertex_colors();
	mesh_.request_vertex_texcoords2D();

	consoleLog_->info("Loading mesh from file '" + filename + "'");
	if (OpenMesh::IO::read_mesh(mesh_, filename, opt_))
	{
		// update face and vertex normals
		if (!opt_.check(OpenMesh::IO::Options::FaceNormal))
		{
			mesh_.update_face_normals();
		}
		else
		{
			consoleLog_->debug("File provides face normals");
		}

		if (!opt_.check(OpenMesh::IO::Options::VertexNormal))
		{
			mesh_.update_vertex_normals();
		}
		else
		{
			consoleLog_->debug("File provides vertex normals");
		}

		// check for possible color information
		if (opt_.check(OpenMesh::IO::Options::VertexColor))
		{
			consoleLog_->debug("File provides vertex colors");
		}
		else
		{
			mesh_.release_vertex_colors();
		}

		if (opt_.check(OpenMesh::IO::Options::FaceColor))
		{
			consoleLog_->debug("File provides face colors");
		}
		else
		{
			mesh_.release_face_colors();
		}

		if (opt_.check(OpenMesh::IO::Options::VertexTexCoord))
		{
			consoleLog_->debug("File provides texture coordinates");
		}

		// loading done
		return true;
	}
	return false;
}

}  // namespace utils