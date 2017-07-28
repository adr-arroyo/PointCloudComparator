/*
 * segmentation.hpp
 *
 *  Created on: 24 jun. 2017
 *      Author: Adrian Arroyo - adr.arroyo.perez@gmail.com

 */

#ifndef INCLUDE_SEGMENTATION_H_
#define INCLUDE_SEGMENTATION_H_
#define PCL_NO_PRECOMPILE

#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

int planar_segmentation(std::string filename);

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> euclidean_cluster_segmentation(std::string filename);

int color_growing_segmentation(std::string filename);

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> region_growing_segmentation(
		std::string filename);

int ground_segmentation(std::string filename);

#endif /* INCLUDE_SEGMENTATION_H_ */
