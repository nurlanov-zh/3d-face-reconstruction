#include <common\data_types.h>
#include <spdlog\spdlog.h>
#include <iostream>

class ProcrustesAligner
{
   public:
	common::Matrix4f estimatePose(
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

		// Scaling
		common::Matrix4f scaling =
			computeScaling(sourcePoints, sourceMean, targetPoints, targetMean);
		std::vector<common::Vector3f> scaledSourcePoints =
			applyScaling(sourcePoints, scaling);

		// Rotation
		common::Matrix3f rotation = estimateRotation(
			scaledSourcePoints, sourceMean, targetPoints, targetMean);

		// Translation
		common::Vector3f translation =
			computeTranslation(sourceMean, targetMean, rotation, scaling);

		// Final transformation
		common::Matrix4f estimatedPose;
		estimatedPose.block(0, 0, 3, 3) = rotation;
		estimatedPose.block(0, 3, 3, 1) = translation;
		estimatedPose.row(3) << 0, 0, 0, 1;
		estimatedPose = estimatedPose * scaling;

		return estimatedPose;
	}

   private:
	std::vector<common::Vector3f> applyScaling(
		const std::vector<common::Vector3f>& points,
		const common::Matrix4f& scaling)
	{
		std::vector<common::Vector3f> scaledPoints = points;

		// Convert Matrix4f to Matrix3f
		common::Matrix3f scaling3f = scaling.block(0, 0, 3, 3);

		// Transform points
		for (int i = 0; i < points.size(); i++)
		{
			scaledPoints[i] = scaling3f * points[i];
		}

		return scaledPoints;
	}

	float computeMeanDistance(const std::vector<common::Vector3f>& points,
							  const common::Vector3f& mean)
	{
		float avgdist = 0.0f;
		float nPoints = points.size();

		for (int i = 0; i < nPoints; i++)
		{
			avgdist += (mean - points[i]).norm();
		}

		avgdist = avgdist / nPoints;

		return avgdist;
	}

	common::Matrix4f computeScaling(
		const std::vector<common::Vector3f>& sourcePoints,
		const common::Vector3f& sourceMean,
		const std::vector<common::Vector3f>& targetPoints,
		const common::Vector3f& targetMean)
	{
		common::Matrix4f scaling = common::Matrix4f::Identity();

		float sourceAvgdist = computeMeanDistance(sourcePoints, sourceMean);
		float targetAvgDist = computeMeanDistance(targetPoints, targetMean);

		float scalingFactor = targetAvgDist / sourceAvgdist;
		scaling = scalingFactor * scaling;
		scaling(3, 3) = 1.0f;

		return scaling;
	}

	common::Vector3f computeMean(const std::vector<common::Vector3f>& points)
	{
		common::Vector3f mean = common::Vector3f::Zero();

		for (int i = 0; i < points.size(); i++)
		{
			mean += points.at(i);
		}

		mean = (1.0 / points.size()) * mean;

		return mean;
	}

	common::Matrix3f estimateRotation(
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

	common::Vector3f computeTranslation(const common::Vector3f& sourceMean,
										const common::Vector3f& targetMean,
										const common::Matrix3f& rotation,
										const common::Matrix4f scaling)
	{
		// Convert Matrix4f to Matrix3f
		common::Matrix3f scaling3f = scaling.block(0, 0, 3, 3);

		common::Vector3f translation =
			(-rotation * scaling3f * sourceMean) + targetMean;

		return translation;
	}
};