#pragma once

#include <common/data_types.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>
#include <sophus/se3.hpp>

namespace matching
{
namespace optimize
{
constexpr size_t NUM_OF_EIG_SHAPE = 160;
constexpr size_t NUM_OF_EIG_EXP = 76;
constexpr size_t NUM_OF_VERTICES = 53490;
constexpr size_t NUM_OF_SPARSE_CORR = 4;
constexpr size_t NUM_OF_LANDMARKS = 68;

struct FaceModelParams
{
	double huber_parameter = 0.5;
	int max_num_iterations = 10;
	int verbosity_level = 1;
};

class FaceModel
{
   public:
	FaceModel(const FaceModelParams& params);
	void setShapeBasis(std::vector<common::float4>& shapeBasis);
	void setShapeBasisDev(std::vector<float>& shapeBasisDev);
	void setExpressionsBasis(std::vector<common::float4>& expressionsBasis);
	void setExpressionsBasisDev(std::vector<float>& expressionsBasisDev);

	void optimize(common::Mesh& neutralMesh, const common::Target& target,
				  const std::vector<common::Vec2i>& correspondences,
				  const common::Matrix4f& poseInit);
	void applyToMesh(common::Mesh& mesh, const common::Matrix4d& poseInit);
	void getWithoutExpressions(common::Mesh& mesh, const common::Matrix4d& poseInit);

   private:
	std::shared_ptr<spdlog::logger> consoleLog_;
	std::shared_ptr<spdlog::logger> errLog_;

	FaceModelParams params_;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> shapeBasis_;
	Eigen::Vector<double, Eigen::Dynamic> shapeBasisDev_;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> expressionsBasis_;
	Eigen::Vector<double, Eigen::Dynamic> expressionsBasisDev_;
	Eigen::Vector<double, Eigen::Dynamic> neutralShape_;
	Eigen::Vector<double, Eigen::Dynamic> shapeBasisCoefs_;
	Eigen::Vector<double, Eigen::Dynamic> expressionsBasisCoefs_;
	Sophus::SE3d transform_;
};
}  // namespace optimize
}  // namespace matching