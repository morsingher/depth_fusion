#include "math_utils.h"

cv::Vec3f ProjectOnCamera(const cv::Vec3f& p_world, const Camera& cam) {

    cv::Vec3f p_cam;
    p_cam[0] = cam.R[0] * p_world[0] + cam.R[1] * p_world[1] + cam.R[2] * p_world[2] + cam.t[0];
    p_cam[1] = cam.R[3] * p_world[0] + cam.R[4] * p_world[1] + cam.R[5] * p_world[2] + cam.t[1];
    p_cam[2] = cam.R[6] * p_world[0] + cam.R[7] * p_world[1] + cam.R[8] * p_world[2] + cam.t[2];

    const float depth = p_cam[2];

    cv::Vec3f pix_depth;
    pix_depth[0] = cam.K[0] * (p_cam[0] / depth) + cam.K[2];
    pix_depth[1] = cam.K[4] * (p_cam[1] / depth) + cam.K[5];
    pix_depth[2] = depth;

    return pix_depth;
}

cv::Vec3f Get3DWorldPoint(const int x, const int y, const float depth, const Camera& cam) {

    cv::Vec3f p_cam, p_world;
    
    // Reprojection
    
    p_cam[0] = depth * (x - cam.K[2]) / cam.K[0];
    p_cam[1] = depth * (y - cam.K[5]) / cam.K[4];
    p_cam[2] = depth;

    // Roto-translation
    
    p_world[0] = cam.R[0] * (p_cam[0] - cam.t[0]) + cam.R[3] * (p_cam[1] - cam.t[1]) + cam.R[6] * (p_cam[2] - cam.t[2]);
    p_world[1] = cam.R[1] * (p_cam[0] - cam.t[0]) + cam.R[4] * (p_cam[1] - cam.t[1]) + cam.R[7] * (p_cam[2] - cam.t[2]);
    p_world[2] = cam.R[2] * (p_cam[0] - cam.t[0]) + cam.R[5] * (p_cam[1] - cam.t[1]) + cam.R[8] * (p_cam[2] - cam.t[2]);

    return p_world;
}

cv::Vec3f RotateNormalToWorld(const cv::Vec3f& normal_c, const float R[9]) {
    cv::Vec3f normal_w;
    normal_w[0] = R[0] * normal_c[0] + R[3] * normal_c[1] + R[6] * normal_c[2];
    normal_w[1] = R[1] * normal_c[0] + R[4] * normal_c[1] + R[7] * normal_c[2];
    normal_w[2] = R[2] * normal_c[0] + R[5] * normal_c[1] + R[8] * normal_c[2];
    return cv::normalize(normal_w);
}
