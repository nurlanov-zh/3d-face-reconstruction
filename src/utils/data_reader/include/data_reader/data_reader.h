#pragma once

#include <common/data_types.h>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/IO/Options.hh>

#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

#include <memory>

namespace utils
{
class DataReader
{
   public:
	DataReader(const std::string& path, OpenMesh::IO::Options opt);

	void readPCAFace();

	const common::Mesh& getMesh() const { return mesh_; }
	common::Mesh& getMesh() { return mesh_; }

   private:
	bool openMesh(const std::string& filename);

   private:
	std::shared_ptr<spdlog::logger> consoleLog_;
	std::shared_ptr<spdlog::logger> errLog_;

	std::string path_;
	OpenMesh::IO::Options opt_;
	common::Mesh mesh_;
};
}  // namespace utils