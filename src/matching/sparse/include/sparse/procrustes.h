#pragma once

#include <common\data_types.h>

namespace matching
{
namespace sparse
{
common::Matrix4f estimatePose(
	const std::vector<common::Vector3f>& sourcePoints,
	const std::vector<common::Vector3f>& targetPoints);

common::Vector3f computeMean(const std::vector<common::Vector3f>& points);

common::Matrix3f estimateRotation(
	const std::vector<common::Vector3f>& sourcePoints,
	const common::Vector3f& sourceMean,
	const std::vector<common::Vector3f>& targetPoints,
	const common::Vector3f& targetMean);

common::Vector3f computeTranslation(const common::Vector3f& sourceMean,
									const common::Vector3f& targetMean,
									const common::Matrix3f& rotation);
}  // namespace sparse
}  // namespace matching
