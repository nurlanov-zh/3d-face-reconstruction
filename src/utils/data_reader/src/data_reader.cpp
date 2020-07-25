#include "data_reader/data_reader.h"

#include <chrono>
#include <experimental/filesystem>
#include <fstream>

namespace utils
{
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
	arnoldCorrespondences_.resize(36);

	landmarkIds_.resize(NUM_OF_LANDMARKS);

	readPCAFace();
	readExpressionsData();
	readAlbedoData();
	readKinectData();
	readCorrespondences();
	readAssignedLandmarks();
	readArnoldFace();

	try
	{
		readRGBD();
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}

	try
	{
		readRealSense();
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
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
		int i = 0;
		for (auto vIt = neutralMesh_.vertices_begin();
			 vIt != neutralMesh_.vertices_end(); ++vIt)
		{
			i++;
			OpenMesh::Vec3f newCoordinate = neutralMesh_.point(*vIt);
			neutralMesh_.set_point(*vIt, newCoordinate * SCALE_NEUTRAL);
		}
		std::cout << i << std::endl;
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

void DataReader::readArnoldFace()
{
	{
		const auto start = std::chrono::steady_clock::now();
		const std::string filename = path_ + "/" + ARNOLD_NAME;
		if (!openMesh(filename, arnoldMesh_))
		{
			errLog_->error("Arnold face is not loaded! Abort!");
			throw std::runtime_error("Arnold face");
		}
		// scale
		for (auto vIt = arnoldMesh_.vertices_begin();
			 vIt != arnoldMesh_.vertices_end(); ++vIt)
		{
			OpenMesh::Vec3f newCoordinate = arnoldMesh_.point(*vIt);
			arnoldMesh_.set_point(*vIt, newCoordinate * 1 / 6.);
		}
		const auto end = std::chrono::steady_clock::now();
		consoleLog_->info(
			"Arnold face is successfully loaded in " +
			std::to_string(
				std::chrono::duration_cast<std::chrono::milliseconds>(end -
																	  start)
					.count()) +
			"ms");
	}

	{
		const std::string filename = path_ + "/" + ARNOLD_CORR_NAME;
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
			in >> arnoldCorrespondences_[x][1] >>
				arnoldCorrespondences_[x][0];  // arnold >> averageMesh
		}

		in.close();
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
		std::string str = entry.path().string();
		{
			const auto pos = str.find(sequenceIdString);
			if (pos == std::string::npos)
			{
				continue;
			}
		}

		{
			const auto pos = str.find(cloudName);
			if (pos != std::string::npos)
			{
				const auto kfpos = str.substr(0, pos).find("_");
				if (pos == std::string::npos)
				{
					continue;
				}

				const auto kf = str.substr(kfpos + 1, pos - kfpos - 1);
				cloudNames_[std::stoi(kf)] = str;
				continue;
			}
		}

		{
			const auto pos = str.find(imageName);
			if (pos != std::string::npos)
			{
				const auto kfpos = str.substr(0, pos).find("_");
				if (pos == std::string::npos)
				{
					continue;
				}

				const auto kf = str.substr(kfpos + 1, pos - kfpos - 1);
				imageNames_[std::stoi(kf)] = str;
				continue;
			}
		}
	}
	if (imageNames_.size() != cloudNames_.size())
	{
		errLog_->error("Different amount of images and clouds");
	}
}

void DataReader::readRealSense()
{
	const std::string path = path_ + "/" + REALSENSE_DIR;

	const std::string cloudName = ".pcd";
	const std::string imageName = ".png";

	for (const auto& entry :
		 std::experimental::filesystem::directory_iterator(path))
	{
		std::string str = entry.path().string();
		{
			const auto pos = str.find(cloudName);
			if (pos != std::string::npos)
			{
				cloudNamesRealSense_.push_back(str);
				continue;
			}
		}

		{
			const auto pos = str.find(imageName);
			if (pos != std::string::npos)
			{
				imageNamesRealSense_.push_back(str);
				continue;
			}
		}
	}

	sort(imageNamesRealSense_.begin(), imageNamesRealSense_.end(),
		 [](const std::string& a, const std::string& b) -> bool {
			 const size_t posA = a.find_last_of("/");
			 const size_t posB = b.find_last_of("/");
			 const size_t posAUnderscore = a.find("_");
			 const size_t posBUnderscore = b.find("_");
			 return std::stoi(a.substr(posA + 1, posAUnderscore)) <=
					std::stoi(b.substr(posB + 1, posBUnderscore));
		 });

	sort(cloudNamesRealSense_.begin(), cloudNamesRealSense_.end(),
		 [](const std::string& a, const std::string& b) -> bool {
			 const size_t posA = a.find_last_of("/");
			 const size_t posB = b.find_last_of("/");
			 const size_t posAUnderscore = a.find("_");
			 const size_t posBUnderscore = b.find("_");
			 return std::stoi(a.substr(posA + 1, posAUnderscore)) <=
					std::stoi(b.substr(posB + 1, posBUnderscore));
		 });

	if (imageNamesRealSense_.size() != cloudNamesRealSense_.size())
	{
		errLog_->error("Different amount of images and clouds");
	}
}

bool DataReader::isNextRGBDExists() const
{
	return imageNames_.size() != 0 && cloudNames_.size() != 0;
}

bool DataReader::isNextRealSenseExists() const
{
	return imageNamesRealSense_.size() != 0 && cloudNamesRealSense_.size() != 0;
}

std::optional<std::pair<cv::Mat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>>
DataReader::nextRGBD()
{
	const auto start = std::chrono::steady_clock::now();
	std::vector<Eigen::Vector3f> landmarks;
	const auto imageName = imageNames_.begin();
	const auto cloudName = cloudNames_.begin();

	const cv::Mat image = cv::imread(imageName->second, cv::IMREAD_COLOR);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
		new pcl::PointCloud<pcl::PointXYZRGB>());
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(cloudName->second, *cloud) == -1)
	{
		errLog_->error("Couldn't read the pcd file: " + cloudName->second);
		return {};
	}

	imageNames_.erase(imageName);
	cloudNames_.erase(cloudName);
	const auto end = std::chrono::steady_clock::now();
	consoleLog_->debug(
		"Next image and cloud are loaded in " +
		std::to_string(
			std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
				.count()) +
		" ms");

	return std::make_optional(std::make_pair(image, cloud));
}

std::optional<std::pair<cv::Mat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>>
DataReader::nextRealSense()
{
	const auto start = std::chrono::steady_clock::now();
	std::vector<Eigen::Vector3f> landmarks;
	const auto imageName = imageNamesRealSense_.begin();
	const auto cloudName = cloudNamesRealSense_.begin();
	consoleLog_->info(*imageName + " " + *cloudName);

	const cv::Mat image = cv::imread(*imageName, cv::IMREAD_COLOR);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
		new pcl::PointCloud<pcl::PointXYZRGB>());
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(*cloudName, *cloud) == -1)
	{
		errLog_->error("Couldn't read the pcd file: " + (*cloudName));
		return {};
	}

	imageNamesRealSense_.erase(imageName);
	cloudNamesRealSense_.erase(cloudName);
	const auto end = std::chrono::steady_clock::now();
	consoleLog_->debug(
		"Next image and cloud are loaded in " +
		std::to_string(
			std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
				.count()) +
		" ms");

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
