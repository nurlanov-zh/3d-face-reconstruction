#pragma once

#include <ceres/ceres.h>
#include <face_model/face_model.h>
#include <Eigen/Core>

struct geomFunctor
{
	geomFunctor(
		const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &shapeBasis,
		const Eigen::Vector<double, Eigen::Dynamic> &shapeBasisDev,
		const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
			&expressionsBasis,
		const Eigen::Vector<double, Eigen::Dynamic> &expressionsBasisDev,
		const Eigen::Vector<double, Eigen::Dynamic> &neutralShape,
		const common::Mesh &targetMesh,
		const std::vector<common::Vec2i> &correspondences);

	template <typename T>
	bool operator()(const T *sShapeBasisCoefs, const T *sExpressionsBasisCoefs,
					const T *sTransform, T *sResiduals) const
	{
		Eigen::Map<Sophus::SE3<T> const> const transform(sTransform);
		Eigen::Map<Eigen::Matrix<T, matching::optimize::NUM_OF_EIG_SHAPE,
								 1> const> const
			shapeBasisCoefs(sShapeBasisCoefs);
		Eigen::Map<
			Eigen::Matrix<T, matching::optimize::NUM_OF_EIG_EXP, 1> const> const
			expressionsBasisCoefs(sExpressionsBasisCoefs);

		auto preVertices = neutralShape_.cast<T>() +
						   shapeBasis_.cast<T>() * shapeBasisCoefs +
						   expressionsBasis_.cast<T>() * expressionsBasisCoefs;

		std::cout << "preVert: " << preVertices.rows() << ", "
				  << preVertices.cols() << std::endl;
		size_t vertId = 0;
		for (common::Mesh::VertexIter vit = targetMesh_.vertices_begin();
			 vit != targetMesh_.vertices_end(); ++vertId, ++vit)
		{
			std::cout << "im inside!" << std::endl;
			T bestDist = T(1e8);
			int bestId = -1;
			const auto point = targetMesh_.point(*vit);
			std::cout << "Point: " << point[0] << ", " << point[1] << ", "
					  << point[2] << std::endl;
			std::cout << preVertices << std::endl;

			for (size_t i = 0; i < matching::optimize::NUM_OF_VERTICES; i++)
			{
				T dist = T(0);
				std::cout << "1" << std::endl;
				std::cout << preVertices(0, 0) << ", " << preVertices(1, 0)
						  << preVertices(2, 0) << std::endl;

				std::cout << "2" << std::endl;
				std::cout << preVertices.block(3 * i, 0, 3, 1) << std::endl;
				Eigen::Vector<T, 3> vertex = preVertices.block(3 * i, 0, 3, 1);
				std::cout << "vertex: " << vertex << std::endl;
				vertex = transform * vertex;
				std::cout << "vertex after: " << vertex << std::endl;

				dist += ceres::pow(T(point[0]) - vertex(0), 2);
				dist += ceres::pow(T(point[1]) - vertex(1), 2);
				dist += ceres::pow(T(point[2]) - vertex(2), 2);
				if (dist < bestDist)
				{
					bestDist = dist;
					bestId = i;
					std::cout << "Best dist: " << bestDist << std::endl;
				}
			}
			sResiduals[vertId] = bestDist;
		}
		return true;
	}
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> shapeBasis_;
	Eigen::Vector<double, Eigen::Dynamic> shapeBasisDev_;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> expressionsBasis_;
	Eigen::Vector<double, Eigen::Dynamic> expressionsBasisDev_;
	Eigen::Vector<double, Eigen::Dynamic> neutralShape_;
	common::Mesh targetMesh_;
	std::vector<common::Vec2i> correspondences_;
};

geomFunctor::geomFunctor(
	const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &shapeBasis,
	const Eigen::Vector<double, -1> &shapeBasisDev,
	const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
		&expressionsBasis,
	const Eigen::Vector<double, -1> &expressionsBasisDev,
	const Eigen::Vector<double, Eigen::Dynamic> &neutralShape,
	const common::Mesh &targetMesh,
	const std::vector<common::Vec2i> &correspondences)
	: shapeBasis_(shapeBasis),
	  shapeBasisDev_(shapeBasisDev),
	  expressionsBasis_(expressionsBasis),
	  expressionsBasisDev_(expressionsBasisDev),
	  neutralShape_(neutralShape),
	  targetMesh_(targetMesh),
	  correspondences_(correspondences)
{
}
