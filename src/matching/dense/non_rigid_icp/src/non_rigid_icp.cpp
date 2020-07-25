#include "non_rigid_icp/non_rigid_icp.h"

#include <chrono>
#include <iostream>

namespace matching
{
namespace refinement
{
NRICP::NRICP(const NRICPParams& params) : params_(params)
{
	consoleLog_ = spdlog::get("console");
	errLog_ = spdlog::get("stderr");

	const auto start = std::chrono::steady_clock::now();

	// stiffness coefs. Equally distributed between init and min value
	const float alphaStep =
		(params.alphaInit - params.alphaMin) / params.numOfOuterIterations;
	for (size_t i = 0; i < params.numOfOuterIterations; i++)
	{
		consoleLog_->debug("Alpha: " +
						   std::to_string(params.alphaInit - i * alphaStep));
		alphas_.push_back(params.alphaInit - i * alphaStep);
	}

	// landmarks coefs. Faded out at the end of optimization as beta = k / i + b
	const float steep =
		params.betaInit / (1 - 1.0 / params.numOfOuterIterations);
	const float shift = params.betaInit - steep;
	for (size_t i = 1; i <= params.numOfOuterIterations; i++)
	{
		consoleLog_->debug("Beta: " + std::to_string(steep / i + shift));
		betas_.push_back(steep / i + shift);
	}

	weights_.resize(params_.numOfVertices, 1);
	ATripletsAlphaMG_ = 0;
	ATripletsWD_ = 8 * params_.numOfEdges;
	ATripletsDL_ = ATripletsWD_ + 4 * params_.numOfVertices;

	const size_t totalSize = ATripletsDL_ + 4 * params_.numOfLandmarks;
	ATriplets_.resize(totalSize);

	A_ = Eigen::SparseMatrix<float>(
		4 * params_.numOfEdges + params_.numOfVertices + params_.numOfLandmarks,
		4 * params_.numOfVertices);

	B_ = Eigen::MatrixX3f::Zero(
		4 * params_.numOfEdges + params_.numOfVertices + params_.numOfLandmarks,
		3);

	XTPrev_ = Eigen::Matrix3Xf(3, 4 * params_.numOfVertices);
	XTPrev_.setZero();
	norm_ = 10e10;

	const auto end = std::chrono::steady_clock::now();

	consoleLog_->debug(
		"Constructed in: " +
		std::to_string(
			std::chrono::duration_cast<std::chrono::microseconds>(end - start)
				.count()) +
		"mcs.");
}

void NRICP::findDeformation(common::Mesh& source, const common::Target& target,
							const std::vector<common::Vec2i>& correspondences)
{
	const auto start = std::chrono::steady_clock::now();
	for (size_t i = 0; i < params_.numOfOuterIterations; ++i)
	{
		size_t iter = 0;
		do
		{
			{
				const auto start = std::chrono::steady_clock::now();

				initTask(alphas_[i], betas_[i], source, target,
						 correspondences);

				const auto end = std::chrono::steady_clock::now();
				consoleLog_->debug(
					"Task initted in: " +
					std::to_string(
						std::chrono::duration_cast<std::chrono::microseconds>(
							end - start)
							.count()) +
					"mcs.");
			}

			{
				const auto start = std::chrono::steady_clock::now();

				optimize(source);

				const auto end = std::chrono::steady_clock::now();
				consoleLog_->debug(
					"Optimized task in: " +
					std::to_string(
						std::chrono::duration_cast<std::chrono::microseconds>(
							end - start)
							.count()) +
					"mcs.");
			}
			iter++;
		} while (norm_ > params_.eps && iter < params_.numOfInnerIterations);
	}
	const auto end = std::chrono::steady_clock::now();
	consoleLog_->debug(
		"Optimized in: " +
		std::to_string(
			std::chrono::duration_cast<std::chrono::microseconds>(end - start)
				.count()) +
		" mcs.");
}

void NRICP::initTask(float alpha, float beta, const common::Mesh& source,
					 const common::Target& target,
					 const std::vector<common::Vec2i>& correspondences)
{
	std::fill(weights_.begin(), weights_.end(), 1.0);

	initBMatrix(source, target);

	initStiffnesMatrix(alpha, source);
	initDistanceMatrix(source);
	initLandmarksMatrix(beta, source, target, correspondences);

	A_.setFromTriplets(ATriplets_.begin(), ATriplets_.end());
}

void NRICP::initStiffnesMatrix(float alpha, const common::Mesh& source)
{
	for (common::Mesh::EdgeIter eit = source.edges_begin();
		 eit != source.edges_end(); ++eit)
	{
		auto vh1 = source.to_vertex_handle(source.halfedge_handle(*eit, 0));
		auto vh2 = source.from_vertex_handle(source.halfedge_handle(*eit, 0));

		ATriplets_[ATripletsAlphaMG_ + 8 * vh1.idx()] =
			Eigen::Triplet<float>(vh1.idx() * 4, vh1.idx() * 4, alpha);

		ATriplets_[ATripletsAlphaMG_ + 8 * vh1.idx() + 1] =
			Eigen::Triplet<float>(vh1.idx() * 4 + 1, vh1.idx() * 4 + 1, alpha);

		ATriplets_[ATripletsAlphaMG_ + 8 * vh1.idx() + 2] =
			Eigen::Triplet<float>(vh1.idx() * 4 + 2, vh1.idx() * 4 + 2, alpha);

		ATriplets_[ATripletsAlphaMG_ + 8 * vh1.idx() + 3] =
			Eigen::Triplet<float>(vh1.idx() * 4 + 3, vh1.idx() * 4 + 3,
								  alpha * params_.gamma);

		ATriplets_[ATripletsAlphaMG_ + 8 * vh2.idx() + 4] =
			Eigen::Triplet<float>(vh2.idx() * 4, vh2.idx() * 4, -alpha);

		ATriplets_[ATripletsAlphaMG_ + 8 * vh2.idx() + 5] =
			Eigen::Triplet<float>(vh2.idx() * 4 + 1, vh2.idx() * 4 + 1, -alpha);

		ATriplets_[ATripletsAlphaMG_ + 8 * vh2.idx() + 6] =
			Eigen::Triplet<float>(vh2.idx() * 4 + 2, vh2.idx() * 4 + 2, -alpha);

		ATriplets_[ATripletsAlphaMG_ + 8 * vh2.idx() + 7] =
			Eigen::Triplet<float>(vh2.idx() * 4 + 3, vh2.idx() * 4 + 3,
								  -alpha * params_.gamma);
	}
}

void NRICP::initLandmarksMatrix(
	float beta, const common::Mesh& source, const common::Target& target,
	const std::vector<common::Vec2i>& correspondences)
{
	for (size_t i = 0; i < correspondences.size(); ++i)
	{
		const auto sourcePoint =
			source.point(OpenMesh::VertexHandle(correspondences[i][0]));
		const auto targetPoint =
			target.mesh.point(OpenMesh::VertexHandle(correspondences[i][1]));

		ATriplets_[ATripletsDL_ + 4 * i] = Eigen::Triplet<float>(
			4 * params_.numOfEdges + params_.numOfVertices + i,
			correspondences[i][0] * 4, beta * sourcePoint[0]);

		ATriplets_[ATripletsDL_ + 4 * i + 1] = Eigen::Triplet<float>(
			4 * params_.numOfEdges + params_.numOfVertices + i,
			correspondences[i][0] * 4 + 1, beta * sourcePoint[1]);

		ATriplets_[ATripletsDL_ + 4 * i + 2] = Eigen::Triplet<float>(
			4 * params_.numOfEdges + params_.numOfVertices + i,
			correspondences[i][0] * 4 + 2, beta * sourcePoint[2]);

		ATriplets_[ATripletsDL_ + 4 * i + 3] = Eigen::Triplet<float>(
			4 * params_.numOfEdges + params_.numOfVertices + i,
			correspondences[i][0] * 4 + 3, beta);

		B_(4 * params_.numOfEdges + params_.numOfVertices + i, 0) =
			beta * targetPoint[0];

		B_(4 * params_.numOfEdges + params_.numOfVertices + i, 1) =
			beta * targetPoint[1];

		B_(4 * params_.numOfEdges + params_.numOfVertices + i, 2) =
			beta * targetPoint[2];
	}
}

void NRICP::initDistanceMatrix(const common::Mesh& source)
{
	for (common::Mesh::VertexIter vit = source.vertices_begin();
		 vit != source.vertices_end(); ++vit)
	{
		const auto idx = vit->idx();
		const auto point = source.point(*vit);
		ATriplets_[ATripletsWD_ + 4 * idx] = Eigen::Triplet<float>(
			4 * params_.numOfEdges + idx, idx * 4, weights_[idx] * point[0]);

		ATriplets_[ATripletsWD_ + 4 * idx + 1] =
			Eigen::Triplet<float>(4 * params_.numOfEdges + idx, idx * 4 + 1,
								  weights_[idx] * point[1]);

		ATriplets_[ATripletsWD_ + 4 * idx + 2] =
			Eigen::Triplet<float>(4 * params_.numOfEdges + idx, idx * 4 + 2,
								  weights_[idx] * point[2]);

		ATriplets_[ATripletsWD_ + 4 * idx + 3] = Eigen::Triplet<float>(
			4 * params_.numOfEdges + idx, idx * 4 + 3, weights_[idx]);
	}
}

void NRICP::initBMatrix(const common::Mesh& source,
						const common::Target& target)
{
	for (common::Mesh::VertexIter vit = source.vertices_begin();
		 vit != source.vertices_end(); ++vit)
	{
		const auto& point = source.point(*vit);
		const float queryPt[3] = {point[0], point[1], point[2]};
		size_t retIndex;
		float outDistSqr;
		nanoflann::KNNResultSet<float> resultSet(1);
		resultSet.init(&retIndex, &outDistSqr);
		const bool found = target.kdTree->findNeighbors(
			resultSet, &queryPt[0], nanoflann::SearchParams(10));

		const auto idx = vit->idx();
		common::PointCloud::Point resPoint = {};
		if (!found)
		{
			weights_[idx] = 0;
		}
		else
		{
			resPoint = target.pc->pts[retIndex];
		}

		if (source.is_boundary(*vit))
		{
			weights_[idx] = 0;
		}
		// set weights if angle between normals above threshold to zero
		// set weights of border vertices to zero
		// set if line segment from Xv to u intersects source

		const auto sourceNormal = source.normal(*vit);
		const auto targetNormal =
			target.mesh.normal(OpenMesh::VertexHandle(retIndex));

		const auto angle = sourceNormal.dot(targetNormal);
		if (angle < params_.normalsThreshold)
		{
			weights_[idx] = 0;
		}

		if (std::sqrt(outDistSqr) > params_.maxHeighborDist)
		{
			weights_[idx] = 0;
		}

		B_(4 * params_.numOfEdges + idx, 0) = weights_[idx] * resPoint.x;

		B_(4 * params_.numOfEdges + idx, 1) = weights_[idx] * resPoint.y;

		B_(4 * params_.numOfEdges + idx, 2) = weights_[idx] * resPoint.z;
	}
}

void NRICP::optimize(common::Mesh& source)
{
	const Eigen::SparseMatrix<float> ATA =
		Eigen::SparseMatrix<float>(A_.transpose()) * A_;
	const Eigen::MatrixX3f ATB =
		Eigen::SparseMatrix<float>(A_.transpose()) * B_;

	Eigen::ConjugateGradient<Eigen::SparseMatrix<float>> solver;
	solver.compute(ATA);
	if (solver.info() != Eigen::Success)
	{
		errLog_->error("Decomposition failed");
		return;
	}

	const Eigen::MatrixX3f X = solver.solve(ATB);
	const Eigen::Matrix3Xf XT = X.transpose();

	for (common::Mesh::VertexIter vit = source.vertices_begin();
		 vit != source.vertices_end(); ++vit)
	{
		if (weights_[vit->idx()] != 0)
		{
			const auto& pointSource = source.point(*vit);
			const Eigen::Vector4f point(pointSource[0], pointSource[1],
										pointSource[2], 1.0f);

			const Eigen::Vector3f pointTransformed =
				XT.block<3, 4>(0, 4 * vit->idx()) * point;

			source.set_point(*vit, {pointTransformed(0), pointTransformed(1),
									pointTransformed(2)});
		}
	}

	norm_ = (XTPrev_ - XT).norm();
	XTPrev_ = XT;
}

}  // namespace refinement
}  // namespace matching