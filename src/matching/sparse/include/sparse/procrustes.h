#pragma once

#include "sparse/procrustes.h"

#include <common/data_types.h>
#include <spdlog/spdlog.h>
#include <iostream>

namespace matching
{
namespace sparse
{
common::Matrix4f estimatePose(
	const std::vector<common::Vector3f>& sourcePoints,
	const std::vector<common::Vector3f>& targetPoints);
}  // namespace sparse
}  // namespace matching