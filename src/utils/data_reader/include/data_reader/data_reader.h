#pragma once

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/IO/Options.hh>

#include <common/data_types.h>

#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

#include <common/data_types.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>

#include <deque>
#include <memory>

namespace utils
{
const std::string NEUTRAL_FACE_NAME = "/averageMesh.off";
const std::string KINECT_DATA_NAME = "/kinectdata.off";
const std::string SHAPE_BASIS_NAME = "/ShapeBasis.matrix";
const std::string SHAPE_BASIS_DEV_NAME = "/StandardDeviationShape.vec";
const std::string EXP_BASIS_NAME = "/ExpressionBasis.matrix";
const std::string EXP_BASIS_DEV_NAME = "/StandardDeviationExpression.vec";
const std::string ALBEDO_BASIS_NAME = "/AlbedoBasis.matrix";
const std::string ALBEDO_BASIS_DEV_NAME = "/StandardDeviationAlbedo.vec";
const std::string SPARSE_CORR_NAME = "/sparse.corr";
const std::string PROCRUSTES_NAME = "/procrustes.off";
const std::string ASSIGNED_LANDMARKS_NAME = "/assigned_landmarks.txt";
const std::string RGBD_DIR = "/RGBD";
const std::string REALSENSE_DIR = "/RealSense";

constexpr size_t NUM_OF_EIG_SHAPE = 160;
constexpr size_t NUM_OF_EIG_EXP = 76;
constexpr size_t NUM_OF_VERTICES = 213960;
constexpr size_t NUM_OF_SPARSE_CORR = 4;
constexpr size_t NUM_OF_LANDMARKS = 68;
constexpr double SCALE_NEUTRAL = 1 / 1000000.0f;

class DataReader
{
   public:
	DataReader(const std::string& path, OpenMesh::IO::Options opt,
			   const int32_t sequenceId = -1);

	const common::Mesh& getNeutralMesh() const { return neutralMesh_; }
	common::Mesh& getNeutralMesh() { return neutralMesh_; }

	const std::vector<common::float4>& getShapeBasis() const
	{
		return shapeBasis_;
	}
	std::vector<common::float4>& getShapeBasis() { return shapeBasis_; }

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
	std::vector<common::float4>& getAlbedoBasis() { return albedoBasis_; }

	const std::vector<float>& getAlbedoBasisDev() const
	{
		return albedoBasisDev_;
	}
	std::vector<float>& getAlbedoBasisDev() { return albedoBasisDev_; }

	const common::Mesh& getKinectMesh() const { return kinectMesh_; }
	common::Mesh& getKinectMesh() { return kinectMesh_; }

	std::vector<common::Vec2i> getCorrespondences() { return correspondences_; }
	common::Mesh& getProcrustesMesh()
	{
		readProcrustes();
		return procrustesMesh_;
	}

	const std::vector<size_t>& getAssignedLandmarks() const
	{
		return landmarkIds_;
	}

	bool isNextRGBDExists() const;
	bool isNextRealSenseExists() const;
	std::optional<std::pair<cv::Mat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>>
	nextRGBD();
	std::optional<std::pair<cv::Mat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>>
	nextRealSense();

   private:
	void readPCAFace();
	void readExpressionsData();
	void readAlbedoData();
	void readKinectData();
	void readCorrespondences();
	void readProcrustes();
	void readRGBD();
	void readRealSense();
	void readAssignedLandmarks();

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
	int32_t sequenceId_;
	common::Mesh neutralMesh_;
	common::Mesh kinectMesh_;
	common::Mesh procrustesMesh_;

	std::vector<common::float4> shapeBasis_;
	std::vector<float> shapeBasisDev_;

	std::vector<common::float4> expressionsBasis_;
	std::vector<float> expressionsBasisDev_;

	std::vector<common::float4> albedoBasis_;
	std::vector<float> albedoBasisDev_;

	std::vector<common::Vec2i> correspondences_;

	std::vector<Eigen::Vector3f> pcdScan_;
	std::map<size_t, std::string> imageNames_;
	std::map<size_t, std::string> cloudNames_;
	std::vector<std::string> imageNamesRealSense_;
	std::vector<std::string> cloudNamesRealSense_;
	std::vector<size_t> landmarkIds_;
};
}  // namespace utils