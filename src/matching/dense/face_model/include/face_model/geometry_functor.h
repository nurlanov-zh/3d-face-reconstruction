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
		const common::Target &target,
		const std::vector<common::Vec2i> &correspondences,
		const Eigen::Matrix4d &poseInit);

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

		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> vertices;
		vertices.resize(3, matching::optimize::NUM_OF_VERTICES);

#pragma omp parallel for
		for (size_t i = 0; i < matching::optimize::NUM_OF_VERTICES; i++)
		{
			vertices.block(0, i, 3, 1) =
				neutralShape_.block(3 * i, 0, 3, 1).cast<T>() +
				shapeBasis_
						.block(3 * i, 0, 3,
							   matching::optimize::NUM_OF_EIG_SHAPE)
						.cast<T>() *
					shapeBasisCoefs +
				expressionsBasis_
						.block(3 * i, 0, 3, matching::optimize::NUM_OF_EIG_EXP)
						.cast<T>() *
					expressionsBasisCoefs;
		}

		std::cout << "vertices: " << vertices.rows() << ", " << vertices.cols()
				  << std::endl;

		for (size_t i = 0; i < target_.mesh.n_vertices(); i++)
		{
			sResiduals[i] = T(10);
		}

#pragma omp parallel for
		for (size_t i = 0; i < matching::optimize::NUM_OF_VERTICES; i++)
		{
			Eigen::Vector<T, 3> vertex = vertices.block(0, i, 3, 1);
			vertex = transform * (poseInitRot_.cast<T>() * vertex +
								  poseInitTrans_.cast<T>());

//			vertex = transform * vertex;

			float queryPt[3];
			if constexpr (std::is_same<T, double>::value)
			{
				queryPt[0] = static_cast<float>(vertex(0));
				queryPt[1] = static_cast<float>(vertex(1));
				queryPt[2] = static_cast<float>(vertex(2));
			}
			else
			{
				queryPt[0] = static_cast<float>(vertex(0).a);
				queryPt[1] = static_cast<float>(vertex(1).a);
				queryPt[2] = static_cast<float>(vertex(2).a);
			}
			size_t retIndex;
			float outDistSqr;
			nanoflann::KNNResultSet<float> resultSet(1);
			resultSet.init(&retIndex, &outDistSqr);
			const bool result = target_.kdTree->findNeighbors(
				resultSet, &queryPt[0], nanoflann::SearchParams(10));

			if (result)
			{
				auto resPoint = target_.pc->pts[retIndex];
				T dist = T(0);
				dist +=
					(T(resPoint.x) - vertex(0)) * (T(resPoint.x) - vertex(0));
				dist +=
					(T(resPoint.y) - vertex(1)) * (T(resPoint.y) - vertex(1));
				dist +=
					(T(resPoint.z) - vertex(2)) * (T(resPoint.z) - vertex(2));

//				if (dist < T(threshold_))
//				{
					if (dist < sResiduals[retIndex])
					{
						sResiduals[retIndex] = dist;
					}
//				}
			}
		}
		return true;
	}
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> shapeBasis_;
	Eigen::Vector<double, Eigen::Dynamic> shapeBasisDev_;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> expressionsBasis_;
	Eigen::Vector<double, Eigen::Dynamic> expressionsBasisDev_;
	Eigen::Vector<double, Eigen::Dynamic> neutralShape_;
	common::Target target_;
	std::vector<common::Vec2i> correspondences_;
	Eigen::Matrix3d poseInitRot_;
	Eigen::Vector3d poseInitTrans_;
	double threshold_ = 10;
};

geomFunctor::geomFunctor(
	const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &shapeBasis,
	const Eigen::Vector<double, -1> &shapeBasisDev,
	const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
		&expressionsBasis,
	const Eigen::Vector<double, -1> &expressionsBasisDev,
	const Eigen::Vector<double, Eigen::Dynamic> &neutralShape,
	const common::Target &target,
	const std::vector<common::Vec2i> &correspondences,
	const Eigen::Matrix4d &poseInit)
	: shapeBasis_(shapeBasis),
	  shapeBasisDev_(shapeBasisDev),
	  expressionsBasis_(expressionsBasis),
	  expressionsBasisDev_(expressionsBasisDev),
	  neutralShape_(neutralShape),
	  target_(target),
	  correspondences_(correspondences)
{
	poseInitRot_ = poseInit.block(0, 0, 3, 3);
	poseInitTrans_ = poseInit.block(0, 3, 3, 1);
}
