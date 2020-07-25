#include <deformation_transfer/deformation_transfer.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <gtest/gtest.h>

using namespace matching::transfer;

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

	typename MeshType::VertexHandle vhandle[4];

	vhandle[0] = mesh.add_vertex(Point(0, 0, 0));
	vhandle[1] = mesh.add_vertex(Point(0, 1, 0));
	vhandle[2] = mesh.add_vertex(Point(2, 0, 0));
	vhandle[3] = mesh.add_vertex(Point(0, 0, 3 - skew));

	std::vector<VertexHandle> face_vhandles;

	face_vhandles.clear();
	face_vhandles.push_back(vhandle[0]);
	face_vhandles.push_back(vhandle[1]);
	face_vhandles.push_back(vhandle[2]);
	mesh.add_face(face_vhandles);

	face_vhandles.clear();
	face_vhandles.push_back(vhandle[3]);
	face_vhandles.push_back(vhandle[1]);
	face_vhandles.push_back(vhandle[0]);
	mesh.add_face(face_vhandles);

	face_vhandles.clear();
	face_vhandles.push_back(vhandle[2]);
	face_vhandles.push_back(vhandle[3]);
	face_vhandles.push_back(vhandle[0]);
	mesh.add_face(face_vhandles);

	face_vhandles.clear();
	face_vhandles.push_back(vhandle[1]);
	face_vhandles.push_back(vhandle[3]);
	face_vhandles.push_back(vhandle[2]);
	mesh.add_face(face_vhandles);


	return mesh.n_vertices();
}

TEST(DeformationTransferTest, deformationTransferKnownCorrTest)
{
	spdlog::set_level(spdlog::level::from_str("trace"));
	spdlog::stdout_color_mt("console");
	spdlog::stderr_color_mt("stderr");

	common::Mesh sourceMesh;
	generateCube<common::Mesh>(sourceMesh, 0.0);

	common::Mesh sourceDeformedMesh;
	generateCube<common::Mesh>(sourceDeformedMesh, 0.3);

	common::Mesh targetMesh;
	generateCube<common::Mesh>(targetMesh, 0.0);

	DeformationTransferParams params;
	auto transfer = DeformationTransfer(params);

	transfer.deformationTransfer(sourceMesh, sourceDeformedMesh, targetMesh);

	for (common::Mesh::VertexIter vit = targetMesh.vertices_begin();
		 vit != targetMesh.vertices_end(); ++vit)
	{
		const auto sourcePoint =
			sourceDeformedMesh.point(OpenMesh::VertexHandle(vit->idx()));
		const auto targetPoint = targetMesh.point(*vit);

		EXPECT_NEAR(sourcePoint[0], targetPoint[0], 1e-1);
		EXPECT_NEAR(sourcePoint[1], targetPoint[1], 1e-1);
		EXPECT_NEAR(sourcePoint[2], targetPoint[2], 1e-1);
	}
}