#ifndef COMMON_H
#define COMMON_H

// C++ STL headers

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <map>
#include <memory>
#include <iomanip>
#include <chrono>

// OpenCV headers

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

struct Camera {
    cv::Mat_<float> K, R, t;
    int height, width;
};

struct Problem {
    int ref_image_id;
    std::vector<int> src_image_ids;
};

#endif // COMMON_H
