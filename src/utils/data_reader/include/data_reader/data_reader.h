#pragma once

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/IO/Options.hh>

#include <common/data_types.h>

#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

#include <memory>

namespace utils
{
class DataReader
{
   public:
	DataReader(const std::string& path, OpenMesh::IO::Options opt);

	const common::Mesh& getNeutralMesh() const { return neutralMesh_; }
	common::Mesh& getNeutralMesh() { return neutralMesh_; }

	const std::vector<common::float4>& getShapeBasis() const
	{
		return shapeBasis_;
	}
	std::vector<common::float4>& getShapeBasis()
	{
		return shapeBasis_;
	}

	const std::vector<float>& getShapeBasisDev() const
	{
		return shapeBasisDev_;
	}
	std::vector<float>& getShapeBasisDev() { return shapeBasisDev_; }

	const std::vector<common::float4>& getExpressionsBasis() const
	{
		return expressionsBasis_;
	}
	std::vector<common::float4>& getExpressionsBasis()
	{
		return expressionsBasis_;
	}

	const std::vector<float>& getExpressionsBasisDev() const
	{
		return expressionsBasisDev_;
	}
	std::vector<float>& getExpressionsBasisDev()
	{
		return expressionsBasisDev_;
	}

	const std::vector<common::float4>& getAlbedoBasis() const
	{
		return albedoBasis_;
	}
	std::vector<common::float4>& getAlbedoBasis()
	{
		return albedoBasis_;
	}

	const std::vector<float>& getAlbedoBasisDev() const
	{
		return albedoBasisDev_;
	}
	std::vector<float>& getAlbedoBasisDev() { return albedoBasisDev_; }

	const common::Mesh& getKinectMesh() const { return kinectMesh_; }
	common::Mesh& getKinectMesh() { return kinectMesh_; }

	std::vector<common::Vec2f> getCorrespondences(){ return correspondences_; }
	common::Mesh& getProcrustesMesh()
	{
		readProcrustes();
		return procrustesMesh_;
	}

   private:
	void readPCAFace();
	void readExpressionsData();
	void readAlbedoData();
	void readKinectData();
	void readCorrespondences();
	void readProcrustes();

	float* loadEigenvectors(const std::string& filename,
							unsigned int components,
							unsigned int numberOfEigenvectors);
	void loadVector(const std::string& filename, float* res,
					unsigned int length);
	bool openMesh(const std::string& filename, common::Mesh& neutralMesh);

	void loadCorrespondences(const std::string& filename);

   private:
	std::shared_ptr<spdlog::logger> consoleLog_;
	std::shared_ptr<spdlog::logger> errLog_;

	std::string path_;
	OpenMesh::IO::Options opt_;
	common::Mesh neutralMesh_;
	common::Mesh kinectMesh_;
	common::Mesh procrustesMesh_;

	std::vector<common::float4> shapeBasis_;
	std::vector<float> shapeBasisDev_;

	std::vector<common::float4> expressionsBasis_;
	std::vector<float> expressionsBasisDev_;

	std::vector<common::float4> albedoBasis_;
	std::vector<float> albedoBasisDev_;

	std::vector<common::Vec2f> correspondences_;
};
}  // namespace utils