#include <non_rigid_icp/non_rigid_icp.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <gtest/gtest.h>

using namespace matching::refinement;

/*
Src                    Target
+-------------+        X-------------X
|             |       X             X
|             |       X             X
|             |       X             X
|             |       X             X
|             |      X             X
|             |      X             X
+-------------+     X-------------X
*/
template <typename MeshType>
size_t generateCube(MeshType& mesh, double skew)
{
	typedef typename MeshType::VertexHandle VertexHandle;
	typedef typename MeshType::Point Point;

	typename MeshType::VertexHandle vhandle[8];

	vhandle[0] = mesh.add_vertex(Point(-1 + skew, -1, 1));
	vhandle[1] = mesh.add_vertex(Point(1 + skew, -1, 1));
	vhandle[2] = mesh.add_vertex(Point(1 + skew, 1, 1));
	vhandle[3] = mesh.add_vertex(Point(-1 + skew, 1, 1));
	vhandle[4] = mesh.add_vertex(Point(-1 - skew, -1, -1));
	vhandle[5] = mesh.add_vertex(Point(1 - skew, -1, -1));
	vhandle[6] = mesh.add_vertex(Point(1 - skew, 1, -1));
	vhandle[7] = mesh.add_vertex(Point(-1 - skew, 1, -1));

	std::vector<VertexHandle> face_vhandles;

	face_vhandles.clear();
	face_vhandles.push_back(vhandle[0]);
	face_vhandles.push_back(vhandle[1]);
	face_vhandles.push_back(vhandle[2]);
	face_vhandles.push_back(vhandle[3]);
	mesh.add_face(face_vhandles);

	face_vhandles.clear();
	face_vhandles.push_back(vhandle[7]);
	face_vhandles.push_back(vhandle[6]);
	face_vhandles.push_back(vhandle[5]);
	face_vhandles.push_back(vhandle[4]);
	mesh.add_face(face_vhandles);

	face_vhandles.clear();
	face_vhandles.push_back(vhandle[1]);
	face_vhandles.push_back(vhandle[0]);
	face_vhandles.push_back(vhandle[4]);
	face_vhandles.push_back(vhandle[5]);
	mesh.add_face(face_vhandles);

	face_vhandles.clear();
	face_vhandles.push_back(vhandle[2]);
	face_vhandles.push_back(vhandle[1]);
	face_vhandles.push_back(vhandle[5]);
	face_vhandles.push_back(vhandle[6]);
	mesh.add_face(face_vhandles);

	face_vhandles.clear();
	face_vhandles.push_back(vhandle[3]);
	face_vhandles.push_back(vhandle[2]);
	face_vhandles.push_back(vhandle[6]);
	face_vhandles.push_back(vhandle[7]);
	mesh.add_face(face_vhandles);

	face_vhandles.clear();
	face_vhandles.push_back(vhandle[0]);
	face_vhandles.push_back(vhandle[3]);
	face_vhandles.push_back(vhandle[7]);
	face_vhandles.push_back(vhandle[4]);
	mesh.add_face(face_vhandles);

	return mesh.n_vertices();
}

TEST(NRICPTest, skewCubeCase)
{
	spdlog::set_level(spdlog::level::from_str("trace"));
	spdlog::stdout_color_mt("console");
	spdlog::stderr_color_mt("stderr");

	common::Mesh sourceMesh;
	generateCube<common::Mesh>(sourceMesh, 0.0);

	common::Mesh targetMesh;
	generateCube<common::Mesh>(targetMesh, 0.2);

	NRICPParams params;
	params.numOfEdges = sourceMesh.n_edges();
	params.numOfVertices = sourceMesh.n_vertices();
	params.numOfLandmarks = 2;
	params.betaInit = 1;
	auto nricp = NRICP(params);

	std::shared_ptr<common::PointCloud> cloud =
		std::make_shared<common::PointCloud>();
	cloud->pts.reserve(targetMesh.n_vertices());
	size_t i = 0;
	for (common::Mesh::VertexIter vit = targetMesh.vertices_begin();
		 vit != targetMesh.vertices_end(); ++i, ++vit)
	{
		const auto point = targetMesh.point(*vit);
		const common::PointCloud::Point pt = {point[0], point[1], point[2]};
		cloud->pts.push_back(pt);
	}

	std::shared_ptr<common::kdTree_t> index =
		std::make_shared<common::kdTree_t>(
			3, *cloud.get(), nanoflann::KDTreeSingleIndexAdaptorParams(10));
	index->buildIndex();

	common::Target target;
	target.kdTree = index;
	target.mesh = targetMesh;

	std::vector<common::Vec2i> correspondences = {{0, 0}, {1, 1}};

	nricp.findDeformation(sourceMesh, target, correspondences);

	for (common::Mesh::VertexIter vit = targetMesh.vertices_begin();
		 vit != targetMesh.vertices_end(); ++vit)
	{
		const auto sourcePoint =
			sourceMesh.point(OpenMesh::VertexHandle(vit->idx()));
		const auto targetPoint = targetMesh.point(*vit);

		EXPECT_FLOAT_EQ(sourcePoint[0], targetPoint[0]);
		EXPECT_FLOAT_EQ(sourcePoint[1], targetPoint[1]);
		EXPECT_FLOAT_EQ(sourcePoint[2], targetPoint[2]);
	}
}