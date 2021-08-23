#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "io_utils.h"
#include "math_utils.h"

using Point = pcl::PointXYZRGBNormal;
using PointCloud = pcl::PointCloud<Point>;
using PointCloudPtr = pcl::PointCloud<Point>::Ptr;

bool GeneratePointCloud(const std::string& data_folder, const std::vector<Problem>& problems, const Options& opt);

#endif // POINT_CLOUD_H