#include "deformation_transfer/deformation_transfer.h"
#include <Eigen/CholmodSupport>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>
#include <iostream>
#include <memory>

namespace matching
{
namespace transfer
{
DeformationTransfer::DeformationTransfer(
	const DeformationTransferParams& params)
	: params_(params)
{
	consoleLog_ = spdlog::get("console");
	errLog_ = spdlog::get("stderr");
}

void DeformationTransfer::applyTransform(
	const common::Mesh& src, common::Mesh& dst,
	const std::vector<common::Vec2i>& correspondences)
{
	buildKDTree(src);

	transform(src, dst, correspondences, params_.wSmoothness, params_.wIdentity,
			  0, 0);

	for (int i = 0; i < 4; ++i)
	{
		transform(src, dst, correspondences, params_.wSmoothness,
				  params_.wIdentity,
				  params_.wClosestMin +
					  i * (params_.wClosestMax - params_.wClosestMin) / 4,
				  1);
	}
}

void DeformationTransfer::transform(
	const common::Mesh& src, common::Mesh& dst,
	const std::vector<common::Vec2i>& correspondences, float wS, float wI,
	float wC, float wCp)
{
	const auto closestCorr = getCorrespondences(src, dst);

	facesNum_ = dst.n_faces();
	const size_t vertices = dst.n_vertices();

	smoothnessTerm_ = 0;
	identityTerm_ = smoothnessTerm_ + 3 * facesNum_;
	closestTerm_ = identityTerm_;
	corrTerm_ = closestTerm_ + closestCorr.size();

	const size_t rows =
		closestCorr.size() + correspondences.size() + 3 * facesNum_;
	const size_t cols = facesNum_ + vertices;

	A_ = Eigen::SparseMatrix<double>(rows, cols);
	A_.reserve(Eigen::VectorXi::Constant(cols, 200));

	B_.clear();
	B_.resize(3);
	B_[0] = Eigen::VectorXd::Zero(rows);
	B_[1] = Eigen::VectorXd::Zero(rows);
	B_[2] = Eigen::VectorXd::Zero(rows);

	setAffine(dst);
	setSmoothness(dst, wS);
	setIdentity(dst, wI);
	setClosest(src, closestCorr, wC);
	setCorresp(src, correspondences, wCp);
	optimize(dst);
}

Eigen::Matrix3d DeformationTransfer::getAffine(
	const common::Mesh& dst, const common::Mesh::FaceHandle& fit)
{
	common::Mesh::FaceEdgeIter fe_it = dst.cfe_iter(fit);
	const auto& vh11 = dst.to_vertex_handle(dst.halfedge_handle(*fe_it, 0));
	const auto& vh12 = dst.from_vertex_handle(dst.halfedge_handle(*fe_it, 0));
	auto vec1 = dst.point(vh12) - dst.point(vh11);
	std::cout << fit.idx() << std::endl;

	++fe_it;

	const auto& vh21 = dst.to_vertex_handle(dst.halfedge_handle(*fe_it, 0));
	const auto& vh22 = dst.from_vertex_handle(dst.halfedge_handle(*fe_it, 0));

	auto vec2 = dst.point(vh22) - dst.point(vh21);

	if (vh21.idx() != vh11.idx())
	{
		vec2 = dst.point(vh21) - dst.point(vh22);
	}
	const auto v4 = vec1.cross(vec2) / vec1.cross(vec2).norm();

	auto vec3 = v4;

	/*if (vec1.norm() < 10e-5 || vec2.norm() < 10e-5 || vec3.norm() < 10e-5 )
	{
		vec1 = common::Mesh::Point(1, 0, 0);
		vec2 = common::Mesh::Point(0, 1, 0);
		vec3 = common::Mesh::Point(0, 0, 1);
	}*/

	//std::cout << "vec " << vec3 << " " << vec2 << " " << vec1 << std::endl;

	Eigen::Matrix3d affine;
	affine.block(0, 0, 3, 1) = Eigen::Vector3d(vec1[0], vec1[1], vec1[2]);
	affine.block(0, 1, 3, 1) = Eigen::Vector3d(vec2[0], vec2[1], vec2[2]);
	affine.block(0, 2, 3, 1) = Eigen::Vector3d(vec3[0], vec3[1], vec3[2]);

	return affine;
}

void DeformationTransfer::setAffine(const common::Mesh& dst)
{
	affineInverse_.clear();

	for (common::Mesh::FaceIter fit = dst.faces_begin(); fit != dst.faces_end();
		 ++fit)
	{
		const Eigen::Matrix3d affine = getAffine(dst, *fit);
		affineInverse_.emplace_back(affine.inverse());
	}
}

void DeformationTransfer::idealTransform(
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& F,
	const common::Mesh& mesh, const common::Mesh& deformedMesh)
{
	F.resize(mesh.n_faces() * 3, 3);
	for (common::Mesh::FaceIter fit = mesh.faces_begin();
		 fit != mesh.faces_end(); ++fit)
	{
				//std::cout << "Affine" << std::endl;

		const Eigen::Matrix3d affine = getAffine(mesh, *fit);

		//std::cout << "Def Affine" << std::endl;
		const Eigen::Matrix3d defformedAffine =
			getAffine(deformedMesh, common::Mesh::FaceHandle(fit->idx()));

		Eigen::MatrixXd Q(3, 3); 
		Eigen::MatrixXd R(3, 3); 
		R.setZero();

		QRFactorize(affine, Q, R);

		Eigen::MatrixXd invRQT = R.inverse() * Q.transpose();
		//std::cout << defformedAffine << std::endl;

		Eigen::MatrixXd Sa = defformedAffine * invRQT;
		F.block(3 * fit->idx(), 0, 3, 3) = Sa.transpose();
	}
}

void DeformationTransfer::deformationTransfer(const common::Mesh& neutralFace,
											  const common::Mesh& src,
											  common::Mesh& dst)
{
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> F;
	idealTransform(F, neutralFace, src);

	const size_t vertices = dst.n_vertices();

	const size_t rows = 3 * dst.n_faces();
	const size_t cols = vertices;

	if (!initialDstCreated_)
	{
		initialDst_ = Eigen::SparseMatrix<double>(rows, cols);
		createInitialDst(dst);
		solver_.compute(initialDst_.transpose() * initialDst_);
	}

	std::cout << "solve failed" << std::endl;
	Eigen::MatrixXd AtF = (initialDst_.transpose()) * F;
	Eigen::MatrixXd x = solver_.solve(AtF);
	//std::cout << F << std::endl;
	std::cout << "solve failed" << std::endl;
	
	//std::cout << initialDst_.transpose() * initialDst_ << std::endl;
	double error[3];
	for (int i = 0; i < 3; ++i) {
		error[i] = x(0, i) - dst.point(common::Mesh::VertexHandle(0))[i];
		for (size_t j = 0; j < dst.n_vertices(); ++j) 
		{
			x(j, i) -= error[i];
		}
	}

	int c = 0;
	for (common::Mesh::VertexIter vit = dst.vertices_begin();
		 vit != dst.vertices_end(); ++vit)
	{
		//std::cout << x(vit->idx(), 0) << " " << x(vit->idx(), 1) << " " << x(vit->idx(), 2) << std::endl;
		dst.set_point(*vit,
					  {x(vit->idx(), 0), x(vit->idx(), 1), x(vit->idx(), 2)});
	}
	std::cout << "ignored " << c << std::endl;
}

void DeformationTransfer::setSmoothness(const common::Mesh& dst, float wS)
{
	size_t row = 0;
	for (common::Mesh::FaceIter fit = dst.faces_begin(); fit != dst.faces_end();
		 ++fit)
	{
		const Eigen::Matrix3d affineHat = affineInverse_[fit->idx()];

		common::Mesh::FaceEdgeIter fe_it_i = dst.cfe_iter(*fit);
		const auto& vhi1 =
			dst.to_vertex_handle(dst.halfedge_handle(*fe_it_i, 0));
		const auto& vhi2 =
			dst.from_vertex_handle(dst.halfedge_handle(*fe_it_i, 0));
		++fe_it_i;
		const auto& vhi3 =
			dst.to_vertex_handle(dst.halfedge_handle(*fe_it_i, 0));

		std::cout << vhi1.idx() << " " << vhi2.idx() << std::endl;

		common::Mesh::FaceFaceIter ff_begin = dst.cff_iter(*fit);
		for (auto ff_it = ff_begin; ff_it.is_valid(); ++ff_it)
		{
			const Eigen::Matrix3d affineHatJ = affineInverse_[ff_it->idx()];

			//std::cout << affineHatJ << " " << affineHat << std::endl;

			common::Mesh::FaceEdgeIter fe_it_j = dst.cfe_iter(*ff_it);
			const auto& vhj1 =
				dst.to_vertex_handle(dst.halfedge_handle(*fe_it_j, 0));
			const auto& vhj2 =
				dst.from_vertex_handle(dst.halfedge_handle(*fe_it_j, 0));
			++fe_it_j;
			const auto& vhj3 =
				dst.to_vertex_handle(dst.halfedge_handle(*fe_it_j, 0));

			for (int k = 0; k < 3; k++)
			{
				A_.coeffRef(row + k, facesNum_ + vhi1.idx()) = 0.0;
				A_.coeffRef(row + k, facesNum_ + vhi2.idx()) = 0.0;
				A_.coeffRef(row + k, facesNum_ + vhi3.idx()) = 0.0;
			}

			for (int k = 0; k < 3; k++)
			{
				A_.coeffRef(smoothnessTerm_ + row + k, fit->idx()) =
					wS * affineHat(0, k);
				A_.coeffRef(smoothnessTerm_ + row + k,
							facesNum_ + vhi1.idx()) +=
					-wS * (affineHat(1, k) + affineHat(2, k));
				A_.coeffRef(smoothnessTerm_ + row + k,
							facesNum_ + vhi2.idx()) += wS * affineHat(1, k);
				A_.coeffRef(smoothnessTerm_ + row + k,
							facesNum_ + vhi3.idx()) += wS * affineHat(2, k);

				A_.coeffRef(smoothnessTerm_ + row + k, ff_begin->idx()) =
					-wS * affineHatJ(0, k);
				A_.coeffRef(smoothnessTerm_ + row + k,
							facesNum_ + vhj1.idx()) +=
					wS * (affineHatJ(1, k) + affineHatJ(2, k));
				A_.coeffRef(smoothnessTerm_ + row + k,
							facesNum_ + vhj2.idx()) += -wS * affineHatJ(1, k);
				A_.coeffRef(smoothnessTerm_ + row + k,
							facesNum_ + vhj3.idx()) += -wS * affineHatJ(2, k);
			}

			row += 3;
		}
	}
}

void DeformationTransfer::createInitialDst(const common::Mesh& dst)
{
	initialDstCreated_ = true;
	typedef Eigen::Triplet<double> Tri;
	std::vector<Tri> tripletList;
	tripletList.reserve(dst.n_faces() * 3);

	for (common::Mesh::FaceIter fit = dst.faces_begin(); fit != dst.faces_end();
		 ++fit)
	{
		common::Mesh::FaceEdgeIter fe_it_i = dst.cfe_iter(*fit);
		const auto& vhi1 =
			dst.to_vertex_handle(dst.halfedge_handle(*fe_it_i, 0));
		const auto& vhi2 =
			dst.from_vertex_handle(dst.halfedge_handle(*fe_it_i, 0));
		++fe_it_i;
		auto vhi3 =
			dst.to_vertex_handle(dst.halfedge_handle(*fe_it_i, 0));
		if (vhi1.idx() == vhi3.idx() || vhi2.idx() == vhi3.idx())
		{
			vhi3 =
				dst.from_vertex_handle(dst.halfedge_handle(*fe_it_i, 0));
		}

		Eigen::Matrix<double, 3, 2, 0, 3, 2> qr;
		Eigen::Matrix<double, 2, 3, 0, 2, 3> rq;

		const auto point1 = dst.point(vhi1);
		const auto point2 = dst.point(vhi2);
		const auto point3 = dst.point(vhi3);
		
		qr.col(0) = Eigen::Vector3d(point2[0] - point1[0], point2[1] - point1[1], point2[2] - point1[2]);
		qr.col(1) = Eigen::Vector3d(point3[0] - point1[0], point3[1] - point1[1], point3[2] - point1[2]);

		Eigen::MatrixXd Q(3, 2);
		Eigen::MatrixXd R(2, 2);
		R.setZero();

		QRFactorize(qr, Q, R);

		Eigen::MatrixXd T = R.inverse() * Q.transpose();
		//std::cout << qr  << std::endl;

		for (int k = 0; k < 3; k++)
		{
			tripletList.push_back(Tri(fit->idx() * 3 + k,
						vhi1.idx(), -(T.coeff(0, k) + T.coeff(1, k))));
			tripletList.push_back(Tri(fit->idx() * 3 + k,
						vhi2.idx(), T.coeff(0, k)));
			tripletList.push_back(Tri(fit->idx() * 3 + k,
						vhi3.idx(), T.coeff(1, k)));
		}
	}
	initialDst_.setFromTriplets(tripletList.begin(), tripletList.end());
}

void DeformationTransfer::setIdentity(const common::Mesh& dst, float wI)
{
	for (size_t i = 0; i < facesNum_; ++i)
	{
		const Eigen::Matrix3d affine = affineInverse_[i];

		common::Mesh::FaceEdgeIter fe_it =
			dst.cfe_iter(common::Mesh::FaceHandle(i));
		const auto& vh1 = dst.to_vertex_handle(dst.halfedge_handle(*fe_it, 0));
		const auto& vh2 =
			dst.from_vertex_handle(dst.halfedge_handle(*fe_it, 0));
		++fe_it;
		const auto& vh3 = dst.to_vertex_handle(dst.halfedge_handle(*fe_it, 0));

		B_[0](identityTerm_) = wI;
		B_[0](identityTerm_ + 1) = 0.0;
		B_[0](identityTerm_ + 2) = 0.0;
		B_[1](identityTerm_) = 0.0;
		B_[1](identityTerm_ + 1) = wI;
		B_[1](identityTerm_ + 2) = 0.0;
		B_[2](identityTerm_) = 0.0;
		B_[2](identityTerm_ + 1) = 0.0;
		B_[2](identityTerm_ + 2) = wI;

		for (int k = 0; k < 3; ++k)
		{
			A_.coeffRef(identityTerm_ + k, i) = wI * affine(0, k);
			A_.coeffRef(identityTerm_ + k, facesNum_ + vh1.idx()) =
				-wI * (affine(1, k) + affine(2, k));
			A_.coeffRef(identityTerm_ + k, facesNum_ + vh2.idx()) =
				wI * affine(1, k);
			A_.coeffRef(identityTerm_ + k, facesNum_ + vh3.idx()) =
				wI * affine(2, k);
		}
	}
}

void DeformationTransfer::setClosest(const common::Mesh& src,
									 const std::vector<common::Vec2i>& corr,
									 float wC)
{
	for (size_t i = 0; i < corr.size(); ++i)
	{
		A_.coeffRef(closestTerm_ + i, facesNum_ + corr[i][0]) = wC;

		const auto point = src.point(common::Mesh::VertexHandle(corr[i][1]));
		B_[0](closestTerm_ + i) = wC * point[0];
		B_[1](closestTerm_ + i) = wC * point[1];
		B_[2](closestTerm_ + i) = wC * point[2];
	}
}

void DeformationTransfer::setCorresp(const common::Mesh& src,
									 const std::vector<common::Vec2i>& corres,
									 float wCp)
{
	for (size_t i = 0; i < corres.size(); ++i)
	{
		A_.coeffRef(corrTerm_ + i, facesNum_ + corres[i][0]) = wCp;
		const auto point = src.point(common::Mesh::VertexHandle(corres[i][1]));
		B_[0](corrTerm_ + i) = wCp * point[0];
		B_[1](corrTerm_ + i) = wCp * point[1];
		B_[2](corrTerm_ + i) = wCp * point[2];
	}
}

std::vector<common::Vec2i> DeformationTransfer::getCorrespondences(
	const common::Mesh& src, const common::Mesh& dst)
{
	std::vector<common::Vec2i> corresp;
	for (common::Mesh::VertexIter vit = dst.vertices_begin();
		 vit != dst.vertices_end(); ++vit)
	{
		const auto& point = dst.point(*vit);
		const float queryPt[3] = {point[0], point[1], point[2]};
		size_t retIndex;
		float outDistSqr;
		nanoflann::KNNResultSet<float> resultSet(1);
		resultSet.init(&retIndex, &outDistSqr);
		const bool result = kdTree_->findNeighbors(resultSet, &queryPt[0],
												   nanoflann::SearchParams(10));

		if (result)
		{
			const auto dstNormal = dst.normal(*vit);
			const auto srcNormal =
				src.normal(common::Mesh::VertexHandle(retIndex));

			if (std::sqrt(outDistSqr) > params_.corrDist)
			{
				continue;
			}

			if (srcNormal.dot(dstNormal) < params_.angleThreshold)
			{
				continue;
			}

			corresp.push_back({vit->idx(), retIndex});
		}
	}
	return corresp;
}

void DeformationTransfer::buildKDTree(const common::Mesh& mesh)
{
	std::shared_ptr<common::PointCloud> cloud =
		std::make_shared<common::PointCloud>();

	cloud->pts.reserve(mesh.n_vertices());

	size_t i = 0;
	for (common::Mesh::VertexIter vit = mesh.vertices_begin();
		 vit != mesh.vertices_end(); ++i, ++vit)
	{
		const auto point = mesh.point(*vit);
		const auto color = mesh.color(*vit);
		const common::PointCloud::Point pt = {point[0], point[1], point[2],
											  color[0], color[1], color[2]};
		cloud->pts.push_back(pt);
	}

	kdTree_.reset(new common::kdTree_t(
		3, *cloud.get(), nanoflann::KDTreeSingleIndexAdaptorParams(10)));
	kdTree_->buildIndex();
}

void DeformationTransfer::optimize(common::Mesh& dst)
{
	const Eigen::SparseMatrix<double> ATA =
		Eigen::SparseMatrix<double>(A_.transpose()) * A_;
	Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>> solver;
	solver.compute(ATA);

	Eigen::VectorXd X[3];
	for (size_t i = 0; i < 3; ++i)
	{
		const Eigen::VectorXd ATB =
			Eigen::SparseMatrix<double>(A_.transpose()) * B_[i];
		X[i] = solver.solve(ATB);
	}

	for (common::Mesh::VertexIter vit = dst.vertices_begin();
		 vit != dst.vertices_end(); ++vit)
	{
		dst.set_point(
			*vit, {X[0](facesNum_ + vit->idx()), X[1](facesNum_ + vit->idx()),
				   X[2](facesNum_ + vit->idx())});
	}
}

void DeformationTransfer::wantQR(Eigen::Matrix<double, 3, 2, 0, 3, 2>& A,
								 Eigen::Matrix<double, 2, 3, 0, 2, 3>& N)
{
	Eigen::Matrix<double, 3, 3, 0, 3, 3> Q, QT;
	Eigen::Matrix<double, 3, 3, 0, 3, 3> H;
	Eigen::Matrix<double, 2, 2, 0, 2, 2> thinR;
	Eigen::Matrix<double, 3, 2, 0, 3, 2> thinQ;
	Eigen::Matrix<double, 3, 2, 0, 3, 2> R;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> E =
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(3, 3);
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> input =
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(3, 3);
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> inputE =
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(3, 3);
	Eigen::Matrix<double, Eigen::Dynamic, 1> I;
	Eigen::Matrix<double, Eigen::Dynamic, 1> u, x, y;
	R = A;
	Q = E;
	H = E;
	for (int i = 0; i < 2; i++)
	{
		x.resize(3 - i, 1);
		y.resize(3 - i, 1);
		input.resize(3 - i, 3 - i);

		I.resize(3 - i, 1);
		I.setZero();
		I(0, 0) = 1;

		inputE.resize(3 - i, 3 - i);
		inputE.setZero();
		for (int j = 0; j < inputE.cols(); j++)
		{
			inputE(j, j) = 1;
		}

		x = R.block(i, i, 3 - i, 1);
		y = x.norm() * I;

		if (x(0, 0) * y(0, 0) > 0) y(0, 0) = y(0, 0) * (-1);

		u = (x - y) / (x - y).norm();

		input = inputE - (2 * (u * u.transpose()));

		H.setZero();
		for (int j = 0; j < H.cols(); j++)
		{
			H(j, j) = 1;
		}

		H.block(i, i, 3 - i, 3 - i) = input;

		R = H * R;
		Q = H * Q;
	}

	QT = Q.transpose();
	thinQ = QT.block(0, 0, 3, 2);
	thinR = R.block(0, 0, 2, 2);
	N = (thinR.inverse()) * (thinQ.transpose());
}

void DeformationTransfer::QRFactorize(const Eigen::MatrixXd &a, Eigen::MatrixXd &q, Eigen::MatrixXd &r)
{
	int i, j, imax, jmax;
	imax = a.rows();
	jmax = a.cols();

	for (j = 0; j<jmax; j++)
	{
		Eigen::VectorXd v(a.col(j));
		for (i = 0; i<j; i++)
		{
			Eigen::VectorXd qi(q.col(i));
			r(i, j) = qi.dot(v);
			v = v - r(i, j)*qi;
		}
		float vv = (float)v.squaredNorm();
		float vLen = sqrtf(vv);
		if (vLen < 10e-8)
		{
			r(j, j) = 1;
			q.col(j).setZero();
		}
		else
		{
			r(j, j) = vLen;
			q.col(j) = v / vLen;
		}
	}
}

}  // namespace transfer
}  // namespace matching