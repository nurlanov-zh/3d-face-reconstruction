#include "data_reader/data_reader.h"

#include <chrono>
#include <experimental/filesystem>
#include <fstream>

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

constexpr size_t NUM_OF_EIG_SHAPE = 160;
constexpr size_t NUM_OF_EIG_EXP = 76;
constexpr size_t NUM_OF_VERTICES = 213960;
constexpr size_t NUM_OF_SPARSE_CORR = 4;
constexpr size_t NUM_OF_LANDMARKS = 68;
constexpr double SCALE_NEUTRAL = 1 / 1000000.0f;

DataReader::DataReader(const std::string& path, OpenMesh::IO::Options opt,
					   const int32_t sequenceId)
	: path_(path), opt_(opt), sequenceId_(sequenceId)
{
	consoleLog_ = spdlog::get("console");
	errLog_ = spdlog::get("stderr");
	shapeBasis_.resize(NUM_OF_EIG_SHAPE * NUM_OF_VERTICES);
	shapeBasisDev_.resize(NUM_OF_EIG_SHAPE);

	expressionsBasis_.resize(NUM_OF_EIG_EXP * NUM_OF_VERTICES);
	expressionsBasisDev_.resize(NUM_OF_EIG_EXP);

	albedoBasis_.resize(NUM_OF_EIG_SHAPE * NUM_OF_VERTICES);
	albedoBasisDev_.resize(NUM_OF_EIG_SHAPE);

	correspondences_.resize(NUM_OF_SPARSE_CORR);

	landmarkIds_.resize(NUM_OF_LANDMARKS);

	readPCAFace();
	readExpressionsData();
	readAlbedoData();
	readKinectData();
	readCorrespondences();
	readAssignedLandmarks();
	readRGBD();
}

void DataReader::readPCAFace()
{
	{
		const auto start = std::chrono::steady_clock::now();
		const std::string filename = path_ + "/" + NEUTRAL_FACE_NAME;
		if (!openMesh(filename, neutralMesh_))
		{
			errLog_->error("Average face is not loaded! Abort!");
			throw std::runtime_error("Average face");
		}
		// scale
		for (auto vIt = neutralMesh_.vertices_begin();
			 vIt != neutralMesh_.vertices_end(); ++vIt)
		{
			OpenMesh::Vec3f newCoordinate = neutralMesh_.point(*vIt);
			neutralMesh_.set_point(*vIt, newCoordinate * SCALE_NEUTRAL);
		}

		const auto end = std::chrono::steady_clock::now();
		consoleLog_->info(
			"Average face is successfully loaded in " +
			std::to_string(
				std::chrono::duration_cast<std::chrono::milliseconds>(end -
																	  start)
					.count()) +
			"ms");
	}

	{
		const auto start = std::chrono::steady_clock::now();
		const std::string filenameBasisShape = path_ + "/" + SHAPE_BASIS_NAME;
		loadVector(filenameBasisShape,
				   reinterpret_cast<float*>(shapeBasis_.data()),
				   4 * shapeBasis_.size());
		const auto end = std::chrono::steady_clock::now();
		consoleLog_->info(
			"Basis shape is successfully loaded in " +
			std::to_string(
				std::chrono::duration_cast<std::chrono::milliseconds>(end -
																	  start)
					.count()) +
			"ms");
	}

	{
		const auto start = std::chrono::steady_clock::now();
		const std::string filenameStdDevShape =
			path_ + "/" + SHAPE_BASIS_DEV_NAME;
		loadVector(filenameStdDevShape, shapeBasisDev_.data(),
				   NUM_OF_EIG_SHAPE);
		const auto end = std::chrono::steady_clock::now();
		consoleLog_->info(
			"Basis shape dev is successfully loaded in " +
			std::to_string(
				std::chrono::duration_cast<std::chrono::milliseconds>(end -
																	  start)
					.count()) +
			"ms");
	}
}

void DataReader::readExpressionsData()
{
	{
		const auto start = std::chrono::steady_clock::now();
		const std::string filenameBasisExp = path_ + "/" + EXP_BASIS_NAME;
		loadVector(filenameBasisExp,
				   reinterpret_cast<float*>(expressionsBasis_.data()),
				   4 * expressionsBasis_.size());
		const auto end = std::chrono::steady_clock::now();
		consoleLog_->info(
			"Basis expressons is successfully loaded in " +
			std::to_string(
				std::chrono::duration_cast<std::chrono::milliseconds>(end -
																	  start)
					.count()) +
			"ms");
	}

	{
		const auto start = std::chrono::steady_clock::now();
		const std::string filenameStdDevExp = path_ + "/" + EXP_BASIS_DEV_NAME;
		loadVector(filenameStdDevExp, expressionsBasisDev_.data(),
				   NUM_OF_EIG_EXP);
		const auto end = std::chrono::steady_clock::now();
		consoleLog_->info(
			"Basis expressions dev is successfully loaded in " +
			std::to_string(
				std::chrono::duration_cast<std::chrono::milliseconds>(end -
																	  start)
					.count()) +
			"ms");
	}
}

void DataReader::readAlbedoData()
{
	{
		const auto start = std::chrono::steady_clock::now();
		const std::string filenameBasisAlbedo = path_ + "/" + ALBEDO_BASIS_NAME;
		loadVector(filenameBasisAlbedo,
				   reinterpret_cast<float*>(albedoBasis_.data()),
				   4 * albedoBasis_.size());
		const auto end = std::chrono::steady_clock::now();
		consoleLog_->info(
			"Basis albedo is successfully loaded in " +
			std::to_string(
				std::chrono::duration_cast<std::chrono::milliseconds>(end -
																	  start)
					.count()) +
			"ms");
	}

	{
		const auto start = std::chrono::steady_clock::now();
		const std::string filenameStdDevAlbedo =
			path_ + "/" + ALBEDO_BASIS_DEV_NAME;
		loadVector(filenameStdDevAlbedo, albedoBasisDev_.data(),
				   NUM_OF_EIG_SHAPE);
		const auto end = std::chrono::steady_clock::now();
		consoleLog_->info(
			"Basis albedo dev is successfully loaded in " +
			std::to_string(
				std::chrono::duration_cast<std::chrono::milliseconds>(end -
																	  start)
					.count()) +
			"ms");
	}
}

void DataReader::readKinectData()
{
	const std::string filename = path_ + "/" + KINECT_DATA_NAME;
	if (openMesh(filename, kinectMesh_))
	{
		consoleLog_->info("Kinect mesh successfully loaded");
		return;
	}
	consoleLog_->info("Kinect mesh is not loaded");
}

void DataReader::readCorrespondences()
{
	const auto start = std::chrono::steady_clock::now();
	const std::string filenameCorr = path_ + "/" + SPARSE_CORR_NAME;
	loadCorrespondences(filenameCorr);
	const auto end = std::chrono::steady_clock::now();
	consoleLog_->info(
		"Correnspondences are successfully loaded in " +
		std::to_string(
			std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
				.count()) +
		"ms");
}

void DataReader::readProcrustes()
{
	const std::string filename = path_ + "/" + PROCRUSTES_NAME;
	if (openMesh(filename, procrustesMesh_))
	{
		consoleLog_->info("Procrustes mesh successfully loaded");
		return;
	}
	consoleLog_->info("Procrustes mesh is not loaded");
}

bool DataReader::openMesh(const std::string& filename, common::Mesh& mesh)
{
	consoleLog_->info("Loading mesh from file '" + filename + "'");
	if (OpenMesh::IO::read_mesh(mesh, filename, opt_))
	{
		mesh.request_face_normals();
		mesh.request_face_colors();
		mesh.request_vertex_normals();
		mesh.request_vertex_colors();
		mesh.request_vertex_texcoords2D();
		
		// update face and vertex normals
		if (!opt_.check(OpenMesh::IO::Options::FaceNormal))
		{
			mesh.update_face_normals();
		}
		else
		{
			consoleLog_->debug("File provides face normals");
		}

		if (!opt_.check(OpenMesh::IO::Options::VertexNormal))
		{
			mesh.update_vertex_normals();
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
			mesh.release_vertex_colors();
		}

		if (opt_.check(OpenMesh::IO::Options::FaceColor))
		{
			consoleLog_->debug("File provides face colors");
		}
		else
		{
			mesh.release_face_colors();
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

void DataReader::loadCorrespondences(const std::string& filename)
{
	std::ifstream in(filename);

	if (!in)
	{
		errLog_->error("ERROR:\tCan not open file: " + filename);
		return;
	}

	size_t length = 0;
	in >> length;

	for (size_t x = 0; x < length; x++)
	{
		in >> correspondences_[x][1] >>
			correspondences_[x][0];  // kinectdata >> averageMesh
	}

	in.close();
}

void DataReader::loadVector(const std::string& filename, float* res,
							unsigned int length)
{
	std::ifstream in(filename, std::ifstream::in | std::ifstream::binary);
	if (!in)
	{
		errLog_->error("ERROR:\tCan not open file: " + filename);
		return;
	}

	unsigned int numberOfEntries;
	in.read((char*)&numberOfEntries, sizeof(unsigned int));
	if (length == 0)
	{
		length = numberOfEntries;
	}
	in.read((char*)(res), length * sizeof(float));

	in.close();
}

void DataReader::readRGBD()
{
	if (sequenceId_ < 0)
	{
		consoleLog_->warn(
			"RGBD scan won't be loaded. sequenceId is less than zero");
		return;
	}

	const std::string path = path_ + "/" + RGBD_DIR;
	std::string sequenceIdString = std::to_string(sequenceId_);
	while (sequenceIdString.size() != 3)
	{
		sequenceIdString = "0" + sequenceIdString;
	}

	const std::string cloudName = "_cloud.pcd";
	const std::string imageName = "_image.png";

	for (const auto& entry :
		 std::experimental::filesystem::directory_iterator(path))
	{
		{
			const auto pos = entry.path().string().find(sequenceIdString);
			if (pos == std::string::npos)
			{
				continue;
			}
		}

		{
			const auto pos = entry.path().string().find(cloudName);
			if (pos != std::string::npos)
			{
				cloudNames_.push_back(entry.path().string());
				continue;
			}
		}

		{
			const auto pos = entry.path().string().find(imageName);
			if (pos != std::string::npos)
			{
				imageNames_.push_back(entry.path().string());
				continue;
			}
		}
	}
	if (imageNames_.size() != cloudNames_.size())
	{
		errLog_->error("Different amount of images and clouds");
	}
}

bool DataReader::isNextRGBDExists() const
{
	return imageNames_.size() != 0 && cloudNames_.size() != 0;
}

std::optional<std::pair<cv::Mat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>>
DataReader::nextRGBD()
{
	std::vector<Eigen::Vector3f> landmarks;
	const auto imageName = imageNames_.front();
	imageNames_.pop_front();

	const auto cloudName = cloudNames_.front();
	cloudNames_.pop_front();

	const cv::Mat image = cv::imread(imageName, cv::IMREAD_COLOR);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
		new pcl::PointCloud<pcl::PointXYZRGB>());
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(cloudName, *cloud) == -1)
	{
		errLog_->error("Couldn't read the pcd file: " + cloudName);
		return {};
	}

	return std::make_optional(std::make_pair(image, cloud));
}

void DataReader::readAssignedLandmarks()
{
	const auto start = std::chrono::steady_clock::now();
	const std::string filenameLandmarks = path_ + "/" + ASSIGNED_LANDMARKS_NAME;

	std::ifstream in(filenameLandmarks);

	if (!in)
	{
		errLog_->error("ERROR:\tCan not open file: " + filenameLandmarks);
		return;
	}

	size_t length = 0;
	in >> length;

	for (size_t x = 0; x < length; x++)
	{
		in >> landmarkIds_[x];
	}

	in.close();

	const auto end = std::chrono::steady_clock::now();
	consoleLog_->info(
		"Assigned landmark ids are successfully loaded in " +
		std::to_string(
			std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
				.count()) +
		"ms");
}

}  // namespace utils
