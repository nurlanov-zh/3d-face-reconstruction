#include "face_model/face_model.h"
#include "face_model/geometry_functor.h"

#include <chrono>
#include <iostream>

namespace matching
{
namespace optimize
{
FaceModel::FaceModel(const FaceModelParams& params) : params_(params)
{
	shapeBasisCoefs_.setZero(NUM_OF_EIG_SHAPE);
	expressionsBasisCoefs_.setZero(NUM_OF_EIG_EXP);
	shapeBasis_.resize(3 * NUM_OF_VERTICES, NUM_OF_EIG_SHAPE);
	shapeBasisDev_.resize(NUM_OF_EIG_SHAPE);
	expressionsBasis_.resize(3 * NUM_OF_VERTICES, NUM_OF_EIG_EXP);
	expressionsBasisDev_.resize(NUM_OF_EIG_EXP);
}

void FaceModel::setShapeBasis(std::vector<common::float4>& shapeBasis)
{
	for (size_t i = 0; i < NUM_OF_EIG_SHAPE; i++)
	{
		for (size_t j = 0; j < NUM_OF_VERTICES; j++)
		{
			shapeBasis_(j * 3, i) = shapeBasis[i * NUM_OF_VERTICES + j].x;
			shapeBasis_(j * 3 + 1, i) = shapeBasis[i * NUM_OF_VERTICES + j].y;
			shapeBasis_(j * 3 + 2, i) = shapeBasis[i * NUM_OF_VERTICES + j].z;
		}
	}
}

void FaceModel::setShapeBasisDev(std::vector<float>& shapeBasisDev)
{
	for (size_t i = 0; i < NUM_OF_EIG_SHAPE; i++)
	{
		shapeBasisDev_(i) = shapeBasisDev[i];
	}
}

void FaceModel::setExpressionsBasis(
	std::vector<common::float4>& expressionsBasis)
{
	for (size_t i = 0; i < NUM_OF_EIG_EXP; i++)
	{
		for (size_t j = 0; j < NUM_OF_VERTICES; j++)
		{
			expressionsBasis_(j * 3, i) =
				expressionsBasis[i * NUM_OF_VERTICES + j].x;
			expressionsBasis_(j * 3 + 1, i) =
				expressionsBasis[i * NUM_OF_VERTICES + j].y;
			expressionsBasis_(j * 3 + 2, i) =
				expressionsBasis[i * NUM_OF_VERTICES + j].z;
		}
	}
}

void FaceModel::setExpressionsBasisDev(std::vector<float>& expressionsBasisDev)
{
	for (size_t i = 0; i < NUM_OF_EIG_EXP; i++)
	{
		expressionsBasisDev_(i) = expressionsBasisDev[i];
	}
}

void FaceModel::optimize(common::Mesh& neutralMesh,
						 const common::Target& target,
						 const std::vector<common::Vec2i>& correspondences,
						 const common::Matrix4f& poseInit)
{
	//	transform_ = Sophus::SE3d(poseInit.cast<double>());

	neutralShape_.resize(3 * NUM_OF_VERTICES);
	size_t i = 0;
	for (common::Mesh::VertexIter vit = neutralMesh.vertices_begin();
		 vit != neutralMesh.vertices_end(); i++, vit++)
	{
		const auto point = neutralMesh.point(*vit);
		neutralShape_(3 * i) = point[0];
		neutralShape_(3 * i + 1) = point[1];
		neutralShape_(3 * i + 2) = point[2];
	}
	ceres::Problem problem;

	auto se3_param = new ceres::ProductParameterization(
		new ceres::EigenQuaternionParameterization(),
		new ceres::IdentityParameterization(3));

	problem.AddParameterBlock(shapeBasisCoefs_.data(), NUM_OF_EIG_SHAPE);
	problem.AddParameterBlock(expressionsBasisCoefs_.data(), NUM_OF_EIG_EXP);
	problem.AddParameterBlock(transform_.data(), Sophus::SE3d::num_parameters,
							  se3_param);

	ceres::CostFunction* cost_function =
		new ceres::AutoDiffCostFunction<geomFunctor, ceres::DYNAMIC, NUM_OF_EIG_SHAPE,
										NUM_OF_EIG_EXP,
										Sophus::SE3d::num_parameters>(
			new geomFunctor(shapeBasis_, shapeBasisDev_, expressionsBasis_,
							expressionsBasisDev_, neutralShape_, target,
							correspondences, poseInit.cast<double>()),
			target.mesh.n_vertices());

	std::cout << "There are " << target.mesh.n_vertices() << " target points"
			  << std::endl;

	std::cout << "There are " << matching::optimize::NUM_OF_VERTICES
			  << " vertices on mesh" << std::endl;

	problem.AddResidualBlock(cost_function,
							 new ceres::HuberLoss(params_.huber_parameter),
							 shapeBasisCoefs_.data(),
							 expressionsBasisCoefs_.data(), transform_.data());

	// Solve
	ceres::Solver::Options ceres_options;
	ceres_options.max_num_iterations = params_.max_num_iterations;
	ceres_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	ceres_options.num_threads = 1;
	ceres::Solver::Summary summary;

	Solve(ceres_options, &problem, &summary);
	switch (params_.verbosity_level)
	{
		// 0: silent
		case 1:
			std::cout << summary.BriefReport() << std::endl;
			break;
		case 2:
			std::cout << summary.FullReport() << std::endl;
			break;
	}

	applyToMesh(neutralMesh, poseInit.cast<double>());
}

void FaceModel::applyToMesh(common::Mesh& mesh,
							const common::Matrix4d& poseInit)
{
	Eigen::Matrix3d poseInitRot_;
	Eigen::Vector3d poseInitTrans_;
	poseInitRot_ = poseInit.block(0, 0, 3, 3);
	poseInitTrans_ = poseInit.block(0, 3, 3, 1);

	size_t i = 0;
	for (common::Mesh::VertexIter vit = mesh.vertices_begin();
		 vit != mesh.vertices_end(); i++, vit++)
	{
		Eigen::Vector3d vertex =
			neutralShape_.block(3 * i, 0, 3, 1) +
			shapeBasis_.block(3 * i, 0, 3,
							  matching::optimize::NUM_OF_EIG_SHAPE) *
				shapeBasisCoefs_ +
			expressionsBasis_.block(3 * i, 0, 3,
									matching::optimize::NUM_OF_EIG_EXP) *
				expressionsBasisCoefs_;

		vertex = transform_ * (poseInitRot_ * vertex + poseInitTrans_);

		mesh.set_point(*vit, {vertex(0), vertex(1), vertex(2)});
	}
}

}  // namespace optimize
}  // namespace matching