#include "point_cloud.h"

int main(int argc, char** argv) {

    if (argc < 3) {
        std::cout << "Usage: <executable> data_folder config_file" << std::endl;
        return EXIT_FAILURE;
    }

    const auto start = std::chrono::high_resolution_clock::now();

    const std::string data_folder(argv[1]);
    std::vector<Problem> problems = ReadProblemList(data_folder);
    if (problems.empty()) {
        std::cout << "Failed to read problem list!" << std::endl;
        return EXIT_FAILURE;
    }

    Options opt;
    if (!ReadOptions(argv[2], opt)) {
        std::cout << "Failed to read options!" << std::endl;
        return EXIT_FAILURE;
    }
    
    if (!GeneratePointCloud(data_folder, problems, opt)) {
        std::cout << "Failed to generate the point cloud!" << std::endl;
        return EXIT_FAILURE;
    }

    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    std::cout << "Total elapsed time: " << duration.count() / 1e9 << " s" << std::endl; 

    return EXIT_SUCCESS;
}
