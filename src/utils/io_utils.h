#ifndef IO_UTILS_H
#define IO_UTILS_H

#include "common.h"

std::vector<Problem> ReadProblemList(const std::string& data_folder);
std::string GetFilename(const std::string& folder, const int id, const std::string& ext);

bool ReadOptions(const char* filename, Options& opt);
bool ReadBinaryMat(const std::string& filename, cv::Mat& mat);
bool ReadColmapMat(const std::string& filename, cv::Mat& mat);
bool ReadCamera(const std::string &filename, Camera& camera);

void RescaleImageAndCamera(const cv::Mat& src, cv::Mat& dst, Camera &camera, const int target_size);

#endif // IO_UTILS_H