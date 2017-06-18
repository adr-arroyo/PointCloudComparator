cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(PointCloudComparator)

find_package(PCL 1.7 REQUIRED COMPONENTS common io)

set(PCL_DIR "/usr/share/pcl-1.7/PCLConfig.cmake")
set(PCL_INCLUDE_DIRS "/usr/include/pcl-1.7")
set(PCL_LIBRARY_DIRS "/usr/lib/")


include_directories(${PCL_INCLUDE_DIRS} "/usr/include/eigen3")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (comparator src/comparator.cpp)
target_link_libraries (comparator ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} "/usr/lib/libpcl_visualization.so" "/usr/lib/libpcl_keypoints.so" "/usr/lib/libpcl_features.so" "/usr/lib/libpcl_kdtree.so" "/usr/lib/libpcl_search.so" "/usr/lib/libpcl_filters.so")
