#include <common\data_types.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/IO/Options.hh>

class SparseAligner
{
   public:
	bool alignSparse(common::Mesh& sourceMesh, common::Mesh& targetMesh,
					 std::vector<common::Vec2f> correspondences);

   private:
	bool transformAndWrite(common::Mesh& sourceMesh,
						  common::Matrix4f estimatedPose);
};

std::ifstream& GotoLine(std::ifstream& file, unsigned int num);

