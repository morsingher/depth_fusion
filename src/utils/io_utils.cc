#include "io_utils.h"

// Read pair.txt file to generate (ref, [src]) pairs

std::vector<Problem> ReadProblemList(const std::string& data_folder) {

    std::vector<Problem> problems;

    const std::string filename = data_folder + std::string("/pair.txt");
    std::ifstream file(filename);
    if (!file) {
        std::cout << "Failed to open neighbors file: " << filename << std::endl;
        return {};
    }

    int num_images;
    file >> num_images;

    for (int i = 0; i < num_images; i++) {

        Problem problem;
        file >> problem.ref_image_id;

        int num_src_images;
        file >> num_src_images;

        for (int j = 0; j < num_src_images; j++) {

            int id;
            float score;
            file >> id >> score;
            if (score > 0.0f) {
                problem.src_image_ids.push_back(id);
            }
        }

        problems.push_back(problem);
    }

    return problems;
}

bool ReadOptions(const char* filename, Options& opt) {
    
    FILE* fp = fopen(filename, "r");
    if (fp == nullptr) {
        std::cout << "Failed to open configuration: " << filename << std::endl;
        return false;
    }

    char read_buffer[65536];
    rapidjson::FileReadStream is(fp, read_buffer, sizeof(read_buffer)); 
    rapidjson::Document d;
    d.ParseStream(is);
    fclose(fp);

    opt.normal_cam = d["normal_cam"].GetBool();
    opt.dyn_cons = d["dyn_cons"].GetBool();
    opt.filter = d["filter"].GetBool();
    opt.refine = d["refine"].GetBool();
    opt.crop = d["crop"].GetBool();
    opt.downsample = d["downsample"].GetBool();

    opt.min_consistent = d["min_consistent"].GetInt();

    opt.max_error = d["max_error"].GetDouble();
    opt.max_diff = d["max_diff"].GetDouble();
    opt.max_angle = d["max_angle"].GetDouble() * M_PI / 180.0;
    opt.max_depth = d["max_depth"].GetDouble();

    return true;
}

bool ReadCamera(const std::string& filename, Camera& camera) {

    std::ifstream file(filename);
    if (!file) {
        std::cout << "Failed to load: " << filename << std::endl;
        return false;
    }

    std::string line;
    file >> line;

    for (int i = 0; i < 3; ++i) {
        file >> camera.R[3 * i + 0] >> camera.R[3 * i + 1] >> camera.R[3 * i + 2] >> camera.t[i];
    }

    float tmp[4];
    file >> tmp[0] >> tmp[1] >> tmp[2] >> tmp[3];
    file >> line;

    for (int i = 0; i < 3; ++i) {
        file >> camera.K[3 * i + 0] >> camera.K[3 * i + 1] >> camera.K[3 * i + 2];
    }

    return true;
}

void RescaleImageAndCamera(const cv::Mat& src, cv::Mat& dst, Camera &camera, const int target_size) {

    if (target_size == std::max(src.cols, src.rows)) {
        dst = src.clone();
        return;
    }

    const float factor_x = static_cast<float>(target_size) / src.cols;
    const float factor_y = static_cast<float>(target_size) / src.rows;
    const float factor = std::min(factor_x, factor_y);

    const int new_cols = std::round(src.cols * factor);
    const int new_rows = std::round(src.rows * factor);

    cv::resize(src, dst, cv::Size(new_cols, new_rows), 0, 0, cv::INTER_LINEAR);

    const float scale_x = new_cols / static_cast<float>(src.cols);
    const float scale_y = new_rows / static_cast<float>(src.rows);

    camera.K[0] *= scale_x;
    camera.K[2] *= scale_x;
    camera.K[4] *= scale_y;
    camera.K[5] *= scale_y;
    camera.width = new_cols;
    camera.height = new_rows;
}

bool ReadColmapMat(const std::string& filename, cv::Mat& mat, bool refine) 
{
    std::fstream text_file(filename, std::ios::in);
    if (!text_file) {
        std::cout << "Failed to open text: " << filename << std::endl;
        return false;
    }

    int rows, cols, channels;
    char dummy;
    text_file >> cols >> dummy >> rows >> dummy >> channels >> dummy;
    std::streampos pos = text_file.tellg();
    text_file.close();

    // Not very nice, I know

    if (channels == 1) {
        mat = cv::Mat::zeros(rows, cols, CV_32FC1);
    } else {
        mat = cv::Mat::zeros(rows, cols, CV_32FC3);
    }

    std::fstream bin_file(filename, std::ios::in | std::ios::binary);
    if (!bin_file) {
        std::cout << "Failed to open bin: " << filename << std::endl;
        return false;
    }
    bin_file.seekg(pos);

    if (!refine) 
    {
        bin_file.read(reinterpret_cast<char*>(mat.data), sizeof(float)*channels*rows*cols);
    }
    else
    {
        std::vector<cv::Mat_<float>> tmp(channels);
        for (int c = 0; c < channels; c++)
        {
            tmp[c] = cv::Mat::zeros(rows, cols, CV_32FC1);
            bin_file.read(reinterpret_cast<char*>(tmp[c].data), sizeof(float)*rows*cols);
        }

        if (channels == 1) {
            mat = tmp[0];
        } else {
            cv::merge(tmp, mat);
        }
    }

    return true;
}

std::string GetFilename(const std::string& folder, const int id, const std::string& ext) {
    std::stringstream path;
    path << folder << "/" << std::setw(8) << std::setfill('0') << id << ext;
    return path.str();
}
