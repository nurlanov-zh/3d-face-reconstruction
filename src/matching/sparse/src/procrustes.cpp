#include "sparse/procrustes.h"
#include <spdlog/spdlog.h>

common::Matrix4f matching::sparse::estimatePose(
	const std::vector<common::Vector3f>& sourcePoints,
	const std::vector<common::Vector3f>& targetPoints)
{
	if (sourcePoints.size() != targetPoints.size())
	{
		spdlog::get("stderr")->error(
			"Number of correspondences does not match!");
		throw std::runtime_error("Number of landmarks");
	}

	// Mean
	auto sourceMean = computeMean(sourcePoints);
	auto targetMean = computeMean(targetPoints);

	// Rotation
	common::Matrix3f rotation =
		estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);

	// Translation
	common::Vector3f translation =
		computeTranslation(sourceMean, targetMean, rotation);

	// Final transformation
	common::Matrix4f estimatedPose;
	estimatedPose.block(0, 0, 3, 3) = rotation;
	estimatedPose.block(0, 3, 3, 1) = translation;
	estimatedPose.row(3) << 0, 0, 0, 1;

	return estimatedPose;
}

common::Vector3f matching::sparse::computeMean(
	const std::vector<common::Vector3f>& points)
{
	common::Vector3f mean = common::Vector3f::Zero();

	for (int i = 0; i < points.size(); i++)
	{
		mean += points.at(i);
	}

	mean = (1.0 / points.size()) * mean;

	return mean;
}

common::Matrix3f matching::sparse::estimateRotation(
	const std::vector<common::Vector3f>& sourcePoints,
	const common::Vector3f& sourceMean,
	const std::vector<common::Vector3f>& targetPoints,
	const common::Vector3f& targetMean)
{
	common::MatrixXf X(sourcePoints.size(), 3);
	common::MatrixXf Y(targetPoints.size(), 3);

	for (int i = 0; i < sourcePoints.size(); i++)
	{
		X.block(i, 0, 1, 3) = (sourcePoints[i] - sourceMean).transpose();
		Y.block(i, 0, 1, 3) = (targetPoints[i] - targetMean).transpose();
	}

	common::MatrixXf transposeY = Y.transpose();
	common::Matrix3f crossCov = transposeY * X;
	common::JacobiSVD<common::MatrixXf> svd(
		crossCov, common::ComputeThinU | common::ComputeThinV);
	common::Matrix3f rotation = svd.matrixU() * svd.matrixV().transpose();
	if (rotation.determinant() == -1)
	{
		common::Matrix3f temp;
		temp << 1, 0, 0, 0, 1, 0, 0, 0, -1;
		rotation = svd.matrixU() * temp * svd.matrixV().transpose();
	}

	return rotation;
}

common::Vector3f matching::sparse::computeTranslation(
	const common::Vector3f& sourceMean, const common::Vector3f& targetMean,
	const common::Matrix3f& rotation)
{
	common::Vector3f translation = (-rotation * sourceMean) + targetMean;

	return translation;
}