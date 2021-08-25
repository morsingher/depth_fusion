# Depth (and Normal) Map Fusion Algorithm

This is a simple C++ implementation of a point cloud generation algorithm from a set of pixelwise depth and normal maps. It depends on PCL and OpenCV. Building the code follows the usual CMake pipeline.

### Input Format

The input is assumed to be the usual Multi-View Stereo (MVS) output: folders with `images`, `cameras`, `depth` and `normal`, along with a `pair.txt` file for view selection. The depth and normal maps are assumed to be saved as the binary output produced by COLMAP.

### Point Cloud Generation

Each image is considered as reference in turn. Each pixel is projected in 3D to the world frame and observed by all its neighbors. Consistent points with low reprojection error, similar normal and close relative depth are stored in the point cloud.

### Options

Besides standard parameters (such as the minimum number of consistent neighbors and the consistency threshold), there are three main options:

- `normal_cam` must be activated if normals are stored in the camera frame.
- `dyn_cons` provides a different consistency check.
- `filter` gives the possibility to save a filtered version of the point cloud, based on the Statistical Outlier Filtering (SOR) method provided by PCL.
- `refine` must be activated if the output of https://github.com/rossimattia/depth-refinement-and-normal-estimation is used for fusion. This is due to a different way they store the normals in binary.

### Output Format

The output is a `point_cloud.ply` file that can be visualized with MeshLab.

### Contributing

Feel free to suggest improvements. The main thing on my to-do-list is to move the fusion algorithm to CUDA, since all the pixels can be processed independently.