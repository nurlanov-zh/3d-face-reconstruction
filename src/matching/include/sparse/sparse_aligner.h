#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/IO/Options.hh>
#include <common\data_types.h>
	class SparseAligner
{
   public:
	bool alignSparse(common::Mesh& sourceMesh,
							  common::Mesh& targetMesh,
							  std::vector<common::Vec2f> correspondences);

   private:
};