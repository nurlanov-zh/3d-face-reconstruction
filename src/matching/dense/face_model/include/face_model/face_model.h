#pragma once

#include <common/data_types.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>

namespace matching
{
namespace optimize
{
struct FaceModelParams
{
};

class FaceModel
{
   public:
	FaceModel(const FaceModelParams& params);

   private:
	std::shared_ptr<spdlog::logger> consoleLog_;
	std::shared_ptr<spdlog::logger> errLog_;

	FaceModelParams params_;
};
}  // namespace optimize
}  // namespace matching