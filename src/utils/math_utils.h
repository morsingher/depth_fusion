#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include "io_utils.h"

cv::Vec3f Get3DWorldPoint(const int x, const int y, const float depth, const Camera& camera);
cv::Vec3f ProjectOnCamera(const cv::Vec3f& p_world, const Camera& camera);
cv::Vec3f RotateNormalToWorld(const cv::Vec3f& n_cam, const cv::Mat& R);

#endif // MATH_UTILS_H