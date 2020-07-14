#include "face_model/face_model.h"

#include <chrono>
#include <iostream>

namespace matching
{
namespace optimize
{
FaceModel::FaceModel(const FaceModelParams& params) : params_(params) {}
}  // namespace optimize
}  // namespace matching