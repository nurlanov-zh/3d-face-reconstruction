#pragma once

#include "common/NearestNeighbor.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
namespace tracking
{
namespace icp
{

const int nIterations = 5;
//const float maxMatchingDist = 0.000015f;
const float outlierMinDistance = 0.14f;


bool trackICP(common::Mesh& sourceMesh, common::Mesh& targetMesh,
			  const std::vector<common::Vec2i>& correspondences);

common::Vector3f corrMean(const std::vector<common::Vec2i>& correspondences,
						  const common::Mesh targetPoints);

void removeOutliers(common::Mesh& mesh, common::Vector3f mean);

common::Matrix4f estimatePose(const std::vector<common::Vector3f> sourceMesh,
							  const std::vector<common::Vector3f> targetMesh);

std::vector<common::Vector3f> transformPoints(
	const std::vector<common::Vector3f>& sourcePoints,
	const common::Matrix4f& pose);

common::Matrix4f estimatePosePointToPoint(
	const std::vector<common::Vector3f>& sourcePoints,
	const std::vector<common::Vector3f>& targetPoints);

bool transformAndWrite(common::Mesh& sourceMesh,
					   const common::Matrix4f& estimatedPose);

/**
 * Helper methods for writing Ceres cost functions.
 */
template <typename T>
static inline void fillVector(const common::Vector3f& input, T* output)
{
	output[0] = T(input[0]);
	output[1] = T(input[1]);
	output[2] = T(input[2]);
}

/**
 * Pose increment is only an interface to the underlying array (in constructor,
 * no copy of the input array is made). Important: Input array needs to have a
 * size of at least 6.
 */
template <typename T>
class PoseIncrement
{
   public:
	explicit PoseIncrement(T* const array) : m_array{array} {}

	void setZero()
	{
		for (int i = 0; i < 6; ++i) m_array[i] = T(0);
	}

	T* getData() const { return m_array; }

	/**
	 * Applies the pose increment onto the input point and produces transformed
	 * output point. Important: The memory for both 3D points (input and output)
	 * needs to be reserved (i.e. on the stack) beforehand).
	 */
	void apply(T* inputPoint, T* outputPoint) const
	{
		// pose[0,1,2] is angle-axis rotation.
		// pose[3,4,5] is translation.
		const T* rotation = m_array;
		const T* translation = m_array + 3;

		T temp[3];
		ceres::AngleAxisRotatePoint(rotation, inputPoint, temp);

		outputPoint[0] = temp[0] + translation[0];
		outputPoint[1] = temp[1] + translation[1];
		outputPoint[2] = temp[2] + translation[2];
	}

	/**
	 * Converts the pose increment with rotation in SO3 notation and translation
	 * as 3D vector into transformation 4x4 matrix.
	 */
	static common::Matrix4f convertToMatrix(const PoseIncrement<double>& poseIncrement)
	{
		// pose[0,1,2] is angle-axis rotation.
		// pose[3,4,5] is translation.
		double* pose = poseIncrement.getData();
		double* rotation = pose;
		double* translation = pose + 3;

		// Convert the rotation from SO3 to matrix notation (with column-major
		// storage).
		double rotationMatrix[9];
		ceres::AngleAxisToRotationMatrix(rotation, rotationMatrix);

		// Create the 4x4 transformation matrix.
		common::Matrix4f matrix;
		matrix.setIdentity();
		matrix(0, 0) = float(rotationMatrix[0]);
		matrix(0, 1) = float(rotationMatrix[3]);
		matrix(0, 2) = float(rotationMatrix[6]);
		matrix(0, 3) = float(translation[0]);
		matrix(1, 0) = float(rotationMatrix[1]);
		matrix(1, 1) = float(rotationMatrix[4]);
		matrix(1, 2) = float(rotationMatrix[7]);
		matrix(1, 3) = float(translation[1]);
		matrix(2, 0) = float(rotationMatrix[2]);
		matrix(2, 1) = float(rotationMatrix[5]);
		matrix(2, 2) = float(rotationMatrix[8]);
		matrix(2, 3) = float(translation[2]);

		return matrix;
	}

   private:
	T* m_array;
};
/**
 * Optimization constraints.
 */
class PointToPointConstraint
{
   public:
	PointToPointConstraint(const common::Vector3f& sourcePoint,
						   const common::Vector3f& targetPoint,
						   const float weight)
		: m_sourcePoint{sourcePoint},
		  m_targetPoint{targetPoint},
		  m_weight{weight}
	{
	}

	template <typename T>
	bool operator()(const T* const pose, T* residuals) const
	{
		// DONE: Implement the point-to-point cost function.
		// The resulting 3D residual should be stored in the residuals array. To
		// apply the pose increment (pose parameters) to the source point, you
		// can use the PoseIncrement class. Important: Ceres automatically
		// squares the cost function.
		T source[3];
		T transformed[3];
		fillVector(m_sourcePoint, source);
		PoseIncrement<T> poseIncrement((T* const)(pose));
		poseIncrement.apply(source, transformed);

		residuals[0] =
			T(m_weight) * T(LAMBDA) * (transformed[0] - T(m_targetPoint(0)));
		residuals[1] =
			T(m_weight) * T(LAMBDA) * (transformed[1] - T(m_targetPoint(1)));
		residuals[2] =
			T(m_weight) * T(LAMBDA) * (transformed[2] - T(m_targetPoint(2)));

		return true;
	}

	static ceres::CostFunction* create(const common::Vector3f& sourcePoint,
									   const common::Vector3f& targetPoint,
									   const float weight)
	{
		return new ceres::AutoDiffCostFunction<PointToPointConstraint, 3, 6>(
			new PointToPointConstraint(sourcePoint, targetPoint, weight));
	}

   protected:
	const common::Vector3f m_sourcePoint;
	const common::Vector3f m_targetPoint;
	const float m_weight;
	const float LAMBDA = 0.1f;
};

}  // namespace icp
}  // namespace tracking