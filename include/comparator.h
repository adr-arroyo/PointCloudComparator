/*
 * comparator.hpp
 *
 *  Created on: 24 jun. 2017
 *      Author: Adrian Arroyo - adr.arroyo.perez@gmail.com

 */

#ifndef INCLUDE_COMPARATOR_H_
#define INCLUDE_COMPARATOR_H_
#define PCL_NO_PRECOMPILE

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/surface/gp3.h>
#include <pcl/features/spin_image.h>
#include <cmath>
#include <pcl/registration/icp.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <vector>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/rift.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types_conversion.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/fpfh.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/octree/octree.h>
#include <pcl/features/shot.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include "../include/segmentation.h"
/*#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Simple_cartesian.h>*/


void processNARF(std::string filename, std::string filename2);

pcl::PointCloud<pcl::PointWithScale> processSift(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz_ptr,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz_ptr2);

void processVHF(std::string filename, std::string filename2);

void processRIFT(std::string filename, std::string filename2);

void processFPFH(std::string filename, std::string filename2);

#endif /* INCLUDE_COMPARATOR_H_ */
