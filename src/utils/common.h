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

// RapidJson headers

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/filereadstream.h>

struct Camera {
    float K[9], R[9], t[3];
    int height, width;
};

struct Problem {
    int ref_image_id;
    std::vector<int> src_image_ids;
};

struct Options {
    bool normal_cam, dyn_cons, filter, refine;
    int min_consistent;
    float max_error, max_diff, max_angle;
};

#endif // COMMON_H
