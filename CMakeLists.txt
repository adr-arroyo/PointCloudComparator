cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project(PointCloudComparator)

find_package(PCL 1.7 REQUIRED COMPONENTS common io)

set(PCL_DIR "/usr/share/pcl-1.7/PCLConfig.cmake")
set(PCL_INCLUDE_DIRS "/usr/include/pcl-1.7")
set(PCL_LIBRARY_DIRS "/usr/lib/")


include_directories(${PCL_INCLUDE_DIRS} "/usr/include/eigen3" "include")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(mylib SHARED src/segmentation.cpp src/comparator.cpp)

add_executable (segmentator src/segmentation.cpp)
target_link_libraries (segmentator mylib ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} "/usr/lib/libpcl_visualization.so" "/usr/lib/libpcl_keypoints.so" "/usr/lib/libpcl_features.so" "/usr/lib/libpcl_kdtree.so" "/usr/lib/libpcl_search.so" "/usr/lib/libpcl_filters.so" "/usr/lib/libpcl_segmentation.so" "/usr/lib/libpcl_sample_consensus.so")

add_executable (comparator src/comparator.cpp)
target_link_libraries (comparator mylib ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} "/usr/lib/libpcl_visualization.so" "/usr/lib/libpcl_keypoints.so" "/usr/lib/libpcl_features.so" "/usr/lib/libpcl_kdtree.so" "/usr/lib/libpcl_search.so" "/usr/lib/libpcl_filters.so" "/usr/lib/libpcl_segmentation.so" "/usr/lib/libpcl_sample_consensus.so")
