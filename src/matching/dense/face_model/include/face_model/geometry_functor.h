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
		const common::Mesh &sourceMesh,
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
			Eigen::Vector<T, 3> vertex =
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
			vertex = transform * (poseInitRot_.cast<T>() * vertex +
								  poseInitTrans_.cast<T>());

			vertices.block(0, i, 3, 1) = vertex;
		}

		std::cout << "vertices: " << vertices.rows() << ", " << vertices.cols()
				  << std::endl;

#pragma omp parallel for
		for (size_t i = 0; i < 2 * target_.mesh.n_vertices(); i++)
		{
			sResiduals[i] = T(1e8);
		}

#pragma omp parallel for
		for (size_t i = 0; i < matching::optimize::NUM_OF_VERTICES; i++)
		{
			Eigen::Vector<T, 3> vertex = vertices.block(0, i, 3, 1);

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
				// Add point to point
				auto resPoint = target_.pc->pts[retIndex];
				auto targetPoint =
					target_.mesh.point(OpenMesh::VertexHandle(retIndex));
				if (std::abs(targetPoint[0] - resPoint.x) > 1e-8 or
					std::abs(targetPoint[1] - resPoint.y) > 1e-8 or
					std::abs(targetPoint[2] - resPoint.z) > 1e-8)
				{
					std::cout << "There is error!" << std::endl;
					std::cout << "Mesh point: " << targetPoint << std::endl;
					std::cout << "Point cloud point: " << resPoint.x << ", " << resPoint.y << ", " <<resPoint.z << std::endl;
				}

				T dist = T(0);
				dist +=
					(T(resPoint.x) - vertex(0)) * (T(resPoint.x) - vertex(0));
				dist +=
					(T(resPoint.y) - vertex(1)) * (T(resPoint.y) - vertex(1));
				dist +=
					(T(resPoint.z) - vertex(2)) * (T(resPoint.z) - vertex(2));

				if (ceres::sqrt(T(wPoint) * dist) < sResiduals[retIndex])
				{
					sResiduals[retIndex] = ceres::sqrt(T(wPoint) * dist);
				}
				// Add point to plain
				auto normalTarget =
					target_.mesh.normal(OpenMesh::VertexHandle(retIndex));
				auto normalSource = sourceMesh_.normal(OpenMesh::VertexHandle(i));

				T distNormalTarget = T(0);
				distNormalTarget += (T(resPoint.x) - vertex(0)) * T(normalTarget[0]);
				distNormalTarget += (T(resPoint.y) - vertex(1)) * T(normalTarget[1]);
				distNormalTarget += (T(resPoint.z) - vertex(2)) * T(normalTarget[2]);

				T distNormalSource = T(0);
				distNormalSource += (T(resPoint.x) - vertex(0)) * T(normalSource[0]);
				distNormalSource += (T(resPoint.y) - vertex(1)) * T(normalSource[1]);
				distNormalSource += (T(resPoint.z) - vertex(2)) * T(normalSource[2]);

				T distNormal = distNormalSource * distNormalSource +
							   distNormalTarget * distNormalTarget;

				if (ceres::sqrt(T(wPlane) * distNormal) <
					sResiduals[retIndex + target_.mesh.n_vertices()])
				{
					sResiduals[retIndex + target_.mesh.n_vertices()] =
						ceres::sqrt(T(wPlane) * distNormal);
				}
			}
		}

#pragma omp parallel for
		for (size_t i = 0; i < 2 * target_.mesh.n_vertices(); i++)
		{
			if (sResiduals[i] >= T(1e8))
			{
				sResiduals[i] = T(0);
			}
		}

		// Add correspondences loss
#pragma omp parallel for
		for (size_t i = 0; i < correspondences_.size(); i++)
		{
			Eigen::Vector<T, 3> vertex =
				vertices.block(0, correspondences_[i][0], 3, 1);
			const auto targetPoint = target_.mesh.point(
				OpenMesh::VertexHandle(correspondences_[i][1]));
			T dist = T(0);
			dist += (T(targetPoint[0]) - vertex(0)) *
					(T(targetPoint[0]) - vertex(0));
			dist += (T(targetPoint[1]) - vertex(1)) *
					(T(targetPoint[1]) - vertex(1));
			dist += (T(targetPoint[2]) - vertex(2)) *
					(T(targetPoint[2]) - vertex(2));
			sResiduals[2 * target_.mesh.n_vertices() + i] = ceres::sqrt(T(wLan) * dist);
		}

		// Add regularizers
#pragma omp parallel for
		for (size_t i = 0; i < matching::optimize::NUM_OF_EIG_SHAPE; ++i)
		{
			sResiduals[2 * target_.mesh.n_vertices() + correspondences_.size() +
					   i] = T(std::sqrt(wReg)) * shapeBasisCoefs(i) / T(shapeBasisDev_(i));
		}

		for (size_t i = 0; i < matching::optimize::NUM_OF_EIG_EXP; ++i)
		{
			sResiduals[2 * target_.mesh.n_vertices() + correspondences_.size() +
				matching::optimize::NUM_OF_EIG_SHAPE + i] = T(std::sqrt(wReg)) *
				expressionsBasisCoefs(i) / T(expressionsBasisDev_(i));
		}

		return true;
	}
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> shapeBasis_;
	Eigen::Vector<double, Eigen::Dynamic> shapeBasisDev_;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> expressionsBasis_;
	Eigen::Vector<double, Eigen::Dynamic> expressionsBasisDev_;
	Eigen::Vector<double, Eigen::Dynamic> neutralShape_;
	common::Mesh sourceMesh_;
	common::Target target_;
	std::vector<common::Vec2i> correspondences_;
	Eigen::Matrix3d poseInitRot_;
	Eigen::Vector3d poseInitTrans_;
	double threshold_ = 10;
	double wPoint = 2;
	double wPlane = 10;
	double wReg = 0.025;
	double wLan = 0.125;
};

geomFunctor::geomFunctor(
	const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &shapeBasis,
	const Eigen::Vector<double, -1> &shapeBasisDev,
	const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
		&expressionsBasis,
	const Eigen::Vector<double, -1> &expressionsBasisDev,
	const Eigen::Vector<double, Eigen::Dynamic> &neutralShape,
	const common::Mesh &sourceMesh,
	const common::Target &target,
	const std::vector<common::Vec2i> &correspondences,
	const Eigen::Matrix4d &poseInit)
	: shapeBasis_(shapeBasis),
	  shapeBasisDev_(shapeBasisDev),
	  expressionsBasis_(expressionsBasis),
	  expressionsBasisDev_(expressionsBasisDev),
	  neutralShape_(neutralShape),
	  sourceMesh_(sourceMesh),
	  target_(target),
	  correspondences_(correspondences)
{
	poseInitRot_ = poseInit.block(0, 0, 3, 3);
	poseInitTrans_ = poseInit.block(0, 3, 3, 1);
}
