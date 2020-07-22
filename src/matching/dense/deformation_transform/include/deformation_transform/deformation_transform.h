#pragma once

#include <common/data_types.h>
#include <memory>

namespace matching
{
namespace transform
{
struct DeformationTransformParams
{
	float wSmoothness = 1.0;
	float wIdentity = 0.001;
	float wClosestMin = 1.0;
	float wClosestMax = 5000.0;
};

class DeformationTransform
{
   public:
	DeformationTransform(const DeformationTransformParams&, const common::Mesh&);

	void applyTransform(const common::Mesh& src, common::Mesh& dst,
						const std::vector<common::Vec2i>& correspondences);

private:
	void buildKDTree(const common::Mesh&);
	std::vector<common::Vec2i> getCorrespondences(const common::Mesh& src, const common::Mesh& dst);
	void transform(const common::Mesh& src, common::Mesh& dst, float wSmoothness, float wIdentity, float wClosestMin);

   private:
   	std::make_shared<common::kdTree_t> kdTree_;
	DeformationTransformParams params_;
};

}  // namespace transform
}  // namespace matching