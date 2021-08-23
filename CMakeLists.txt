cmake_minimum_required (VERSION 3.5)
project (depth_fusion)

set(CMAKE_CXX_FLAGS "-O3 -ffast-math -march=native -mtune=native")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(OpenCV REQUIRED)
find_package(PCL 1.2 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    src/utils
                    src/fusion)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} -Wno-unused-result -Wno-dev)

add_executable(depth_fusion src/utils/io_utils.cc
                            src/utils/math_utils.cc
                            src/fusion/point_cloud.cc
                            src/main.cc)

target_link_libraries(depth_fusion ${OpenCV_LIBS} ${PCL_LIBRARIES})
