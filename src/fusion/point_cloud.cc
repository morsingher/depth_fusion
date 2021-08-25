#include "point_cloud.h"

bool GeneratePointCloud(const std::string& data_folder, const std::vector<Problem>& problems, const Options& opt) {

    const int num_images = problems.size();

    // Load input data

    std::vector<cv::Mat_<cv::Vec3b>> images;
    std::vector<Camera> cameras;
    std::vector<cv::Mat_<float>> depths;
    std::vector<cv::Mat_<cv::Vec3f>> normals;
    std::vector<cv::Mat_<uchar>> masks;

    std::cout << "Loading input data..." << std::endl;

    for (int i = 0; i < num_images; ++i) {

        std::cout << "Loading image " << i << "..." << std::endl;

        const int ref = problems[i].ref_image_id;

        const std::string image_path = GetFilename(data_folder + "/images/" , ref, ".jpg");
        cv::Mat_<cv::Vec3b> image = cv::imread(image_path, cv::IMREAD_COLOR);

        const std::string cam_path = GetFilename(data_folder + "/cameras/", ref, ".txt");
        Camera camera;
        if (!ReadCamera(cam_path, camera)) {
            std::cout << "Failed to read camera: " << cam_path << std::endl;
            return false;
        };

        std::string depth_path;
        if (opt.refine) {
            depth_path = GetFilename(data_folder + "/depth_refined/", ref, ".dmb");
        } else {
            depth_path = GetFilename(data_folder + "/depth/", ref, ".dmb");
        }

        cv::Mat_<float> depth;
        if (!ReadColmapMat(depth_path, depth, opt.refine)) {
            std::cout << "Failed to read depth: " << depth_path << std::endl;
            return false;
        }
        depths.push_back(depth);

        std::string normal_path;
        if (opt.refine) {
            normal_path = GetFilename(data_folder + "/normal_refined/", ref, ".dmb");
        } else {
            normal_path = GetFilename(data_folder + "/normal/", ref, ".dmb");
        }

        cv::Mat_<cv::Vec3f> normal;
        if (!ReadColmapMat(normal_path, normal, opt.refine)) {
            std::cout << "Failed to read normal: " << normal_path << std::endl;
            return false;
        }
        normals.push_back(normal);

        const int target_size = std::max(depth.cols, depth.rows);
        cv::Mat_<cv::Vec3b> scaled_image;
        RescaleImageAndCamera(image, scaled_image, camera, target_size);
        images.push_back(scaled_image);
        cameras.push_back(camera);
        
        masks.push_back(cv::Mat::zeros(depth.rows, depth.cols, CV_8UC1));
    }

    PointCloud point_cloud;

    for (int i = 0; i < num_images; ++i) {

        std::cout << "Processing image " << i << "..." << std::endl;
        
        const int cols = depths[i].cols;
        const int rows = depths[i].rows;
        const int num_ngb = problems[i].src_image_ids.size();
        std::vector<cv::Point2i> used_list(num_ngb, cv::Point2i(-1, -1));
        
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {

                if (masks[i](r, c) != 1) {

                    const float ref_depth = depths[i](r, c);
                    cv::Vec3f ref_normal = normals[i](r, c);
                    if (opt.normal_cam) {
                        ref_normal = RotateNormalToWorld(ref_normal, cameras[i].R);
                    }

                    if (ref_depth > 0.0)
                    {
                        const cv::Vec3f ref_world_point = Get3DWorldPoint(c, r, ref_depth, cameras[i]);
                        cv::Vec3f consistent_point = ref_world_point;
                        cv::Vec3f consistent_normal = ref_normal;
                        cv::Vec3f consistent_color = images[i](r, c);

                        int num_consistent = 0;
                        float dynamic_consistency = 0.0f;

                        for (int j = 0; j < num_ngb; ++j) {

                            const int src_id = problems[i].src_image_ids[j];
                            const int src_cols = depths[src_id].cols;
                            const int src_rows = depths[src_id].rows;

                            const cv::Vec3f src_pix_depth = ProjectOnCamera(ref_world_point, cameras[src_id]);
                            const int src_r = static_cast<int>(src_pix_depth[1] + 0.5f);
                            const int src_c = static_cast<int>(src_pix_depth[0] + 0.5f);
                            
                            if (src_c >= 0 && src_c < src_cols && src_r >= 0 && src_r < src_rows && masks[src_id](src_r, src_c) != 1) 
                            {
                                const float src_depth = depths[src_id](src_r, src_c);
                                cv::Vec3f src_normal = normals[src_id](src_r, src_c);
                                if (opt.normal_cam) {
                                    src_normal = RotateNormalToWorld(src_normal, cameras[src_id].R);
                                }
                                
                                if (src_depth > 0.0)
                                {
                                    const cv::Vec3f src_world_point = Get3DWorldPoint(src_c, src_r, src_depth, cameras[src_id]);
                                    const cv::Vec3f ref_pix_depth = ProjectOnCamera(src_world_point, cameras[i]);

                                    const float reproj_error = std::sqrt(std::pow(c - ref_pix_depth[0], 2) + std::pow(r - ref_pix_depth[1], 2));
                                    const float relative_depth_diff = std::fabs(ref_pix_depth[2] - ref_depth) / ref_depth;
                                    const float angle = std::acos(ref_normal.dot(src_normal));

                                    if (reproj_error < opt.max_error && relative_depth_diff < opt.max_diff && angle < opt.max_angle) 
                                    {
                                        if (!opt.dyn_cons) {
                                            consistent_point += src_world_point;
                                            consistent_normal += src_normal;
                                            consistent_color += images[src_id](src_r, src_c);
                                        }

                                        used_list[j].x = src_c;
                                        used_list[j].y = src_r;
                                        num_consistent++;
                                        dynamic_consistency += std::exp(- 200.0 * relative_depth_diff - 10.0 * angle - reproj_error);
                                    }
                                }
                            }
                        }

                        if ((!opt.dyn_cons && num_consistent >= opt.min_consistent ||
                            (opt.dyn_cons && num_consistent >= 1 && dynamic_consistency > 0.3 * num_consistent)))
                        {
                            if (!opt.dyn_cons) {
                                consistent_point /= (num_consistent + 1.0f);
                                consistent_normal /= (num_consistent + 1.0f);
                                consistent_color /= (num_consistent + 1.0f);
                            }

                            Point point3D;
                            point3D.x = consistent_point[0];
                            point3D.y = consistent_point[1];
                            point3D.z = consistent_point[2];
                            point3D.r = static_cast<int>(consistent_color[2]);
                            point3D.g = static_cast<int>(consistent_color[1]);
                            point3D.b = static_cast<int>(consistent_color[0]);
                            point3D.normal_x = consistent_normal[0];
                            point3D.normal_y = consistent_normal[1];
                            point3D.normal_z = consistent_normal[2];
                            
                            point_cloud.push_back(point3D);

                            for (int j = 0; j < num_ngb; ++j) {
                                if (used_list[j].x != -1) {
                                    masks[problems[i].src_image_ids[j]](used_list[j].y, used_list[j].x) = 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    std::cout << "Saving the point cloud..." << std::endl;
    std::cout << "Number of points: " << point_cloud.points.size() << std::endl;

    std::string ply_path = data_folder + "/point_cloud.ply";
    pcl::io::savePLYFileBinary(ply_path, point_cloud);

    // Filtering

    if (opt.filter) {
        std::cout << "Filtering the point cloud..." << std::endl;

        PointCloudPtr point_cloud_filt(new PointCloud);
        PointCloudPtr point_cloud_ptr(new PointCloud);
        *point_cloud_ptr = point_cloud;

        pcl::StatisticalOutlierRemoval<Point> sor;
        sor.setInputCloud(point_cloud_ptr);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*point_cloud_filt);

        std::cout << "Saving the filtered point cloud..." << std::endl;

        ply_path = data_folder + "/point_cloud_filtered.ply";
        pcl::io::savePLYFileBinary(ply_path, *point_cloud_filt);
    }

    std::cout << "Done!" << std::endl;

    return true;
}
