#pragma once

#include <common/data_types.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <memory>

namespace matching
{
namespace transfer
{
struct DeformationTransferParams
{
	float wSmoothness = 1.0;
	float wIdentity = 0.001;
	float wClosestMin = 1.0;
	float wClosestMax = 5000.0;
	float wCorrespondence = 10.0;
	float angleThreshold = 0.7;
	float corrDist = 0.2;
};

class DeformationTransfer
{
   public:
	DeformationTransfer(const DeformationTransferParams&);

	void applyTransform(const common::Mesh&, common::Mesh&,
						const std::vector<common::Vec2i>&);

	void deformationTransfer(const common::Mesh& neutralFace,
							 const common::Mesh& src, common::Mesh& dst);

   private:
	void idealTransform(
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& F,
		const common::Mesh& mesh, const common::Mesh& deformedMesh);
	void setClosest(const common::Mesh&, const std::vector<common::Vec2i>& corr,
					float wC);
	void setIdentity(const common::Mesh&, float wI);
	void setSmoothness(const common::Mesh&, float wS);
	void setCorresp(const common::Mesh&, const std::vector<common::Vec2i>&,
					float wCp);
	void setAffine(const common::Mesh&);
	void buildKDTree(const common::Mesh&);
	std::vector<common::Vec2i> getCorrespondences(const common::Mesh&,
												  const common::Mesh&);
	void transform(const common::Mesh&, common::Mesh&,
				   const std::vector<common::Vec2i>& correspondences,
				   float wSmoothness, float wIdentity, float wClosestMin,
				   float wCorr);
	void optimize(common::Mesh&);
	Eigen::Matrix3d getAffine(const common::Mesh& dst,
							  const common::Mesh::FaceHandle& fit);
	void createInitialDst(const common::Mesh& dst);
	void wantQR(Eigen::Matrix<double, 3, 2, 0, 3, 2>& A,
								 Eigen::Matrix<double, 2, 3, 0, 2, 3>& N);
	void QRFactorize(const Eigen::MatrixXd &a, Eigen::MatrixXd &q, Eigen::MatrixXd &r);

   private:
	std::shared_ptr<spdlog::logger> consoleLog_;
	std::shared_ptr<spdlog::logger> errLog_;

	std::vector<Eigen::Matrix3d> affineInverse_;
	std::vector<Eigen::VectorXd> B_;
	Eigen::SparseMatrix<double> A_;
	Eigen::SparseMatrix<double> initialDst_;
	Eigen::SimplicialCholesky<Eigen::SparseMatrix<double> > solver_;

	std::shared_ptr<common::kdTree_t> kdTree_;
	DeformationTransferParams params_;

	size_t facesNum_ = 0;
	size_t smoothnessTerm_ = 0;
	size_t identityTerm_ = 0;
	size_t closestTerm_ = 0;
	size_t corrTerm_ = 0;

	bool initialDstCreated_ = false;
};

}  // namespace transfer
}  // namespace matching