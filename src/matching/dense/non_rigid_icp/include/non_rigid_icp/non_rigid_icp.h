#pragma once

#include <common/data_types.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>

namespace matching
{
namespace refinement
{
struct NRICPParams
{
	size_t numOfOuterIterations = 8;
	size_t numOfInnerIterations = 3;
	size_t numOfEdges = 20000;
	size_t numOfVertices = 20000;
	size_t numOfLandmarks = 60;
	float gamma = 1;
	float alphaInit = 50;
	float alphaMin = 1;
	float betaInit = 1;
	float normalsThreshold = 0.97;
	float eps = 60;
};

class NRICP
{
   public:
	NRICP(const NRICPParams& params);

	void findDeformation(common::Mesh& source, const common::Target& target,
						 const std::vector<common::Vec2i>& correspondences);

   private:
	void initTask(float alpha, float beta, const common::Mesh& source,
				  const common::Target& target,
				  const std::vector<common::Vec2i>& correspondences);
	void initStiffnesMatrix(float alpha, const common::Mesh& source);
	void initLandmarksMatrix(float beta, const common::Mesh& source,
							 const common::Target& target,
							 const std::vector<common::Vec2i>& correspondences);
	void initDistanceMatrix(const common::Mesh& source);
	void initBMatrix(const common::Mesh& source, const common::Target& target);

	void optimize(common::Mesh& source);

   private:
	std::shared_ptr<spdlog::logger> consoleLog_;
	std::shared_ptr<spdlog::logger> errLog_;

	NRICPParams params_;

	size_t ATripletsAlphaMG_;
	size_t ATripletsWD_;
	size_t ATripletsDL_;

	std::vector<float> alphas_;
	std::vector<float> betas_;

	std::vector<float> weights_;

	std::vector<Eigen::Triplet<float>> ATriplets_;
	Eigen::SparseMatrix<float> A_;
	Eigen::MatrixX3f B_;
	Eigen::Matrix3Xf XTPrev_;
	float norm_;
};
}  // namespace refinement
}  // namespace matching