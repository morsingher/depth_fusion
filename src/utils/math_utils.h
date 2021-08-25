#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include "io_utils.h"

cv::Vec3f Get3DWorldPoint(const int x, const int y, const float depth, const Camera& cam);
cv::Vec3f ProjectOnCamera(const cv::Vec3f& p_world, const Camera& cam);
cv::Vec3f RotateNormalToWorld(const cv::Vec3f& normal_c, const float R[9]);

#endif // MATH_UTILS_H