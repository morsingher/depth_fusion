#include "math_utils.h"

cv::Vec3f ProjectOnCamera(const cv::Vec3f& p_world, const Camera& camera) {

    const cv::Mat_<float> p_cam = camera.R * cv::Mat(p_world) + camera.t;
    const float depth = p_cam(2, 0);

    cv::Vec3f pix_depth;
    pix_depth[0] = camera.K(0, 0) * (p_cam(0, 0) / depth) + camera.K(0, 2);
    pix_depth[1] = camera.K(1, 1) * (p_cam(1, 0) / depth) + camera.K(1, 2);
    pix_depth[2] = depth;

    return pix_depth;
}

cv::Vec3f Get3DWorldPoint(const int x, const int y, const float depth, const Camera& camera) {

    cv::Vec3f p_cam;
    
    // Reprojection
    
    p_cam[0] = depth * (x - camera.K(0, 2)) / camera.K(0, 0);
    p_cam[1] = depth * (y - camera.K(1, 2)) / camera.K(1, 1);
    p_cam[2] = depth;

    // Roto-translation

    const cv::Mat_<float> p_world = camera.R.t() * (cv::Mat(p_cam) - camera.t);
    return cv::Vec3f(p_world(0, 0), p_world(1, 0), p_world(2, 0));
}

cv::Vec3f RotateNormalToWorld(const cv::Vec3f& n_cam, const cv::Mat& R) {
    const cv::Mat_<float> n_world = R.t() * cv::Mat(n_cam);
    return cv::normalize(cv::Vec3f(n_world(0, 0), n_world(1, 0), n_world(2, 0)));
}
