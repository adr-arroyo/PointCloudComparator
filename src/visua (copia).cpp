/*
 * visualizer.cpp
 *
 *  Created on: 20 may. 2017
 *      Author: Adrian Arroyo adr.arroyo.perez@gmail.com
 */

#include <iostream>

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/console/parse.h>

int main(int argc, char** argv) {
	// Fetch point cloud filename in arguments | Works with PCD and PLY files
	std::vector<int> filenames;
	bool file_is_pcd = false;

	filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");

	if (filenames.size() != 1) {
		filenames = pcl::console::parse_file_extension_argument(argc, argv,
				".pcd");

		if (filenames.size() != 1) {
			return -1;
		} else {
			file_is_pcd = true;
		}
	}

	// Load file | Works with PCD and PLY files
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(
			new pcl::PointCloud<pcl::PointXYZ>());

	if (file_is_pcd) {
		if (pcl::io::loadPCDFile(argv[filenames[0]], *source_cloud) < 0) {
			std::cout << "Error loading point cloud " << argv[filenames[0]]
					<< std::endl << std::endl;
			return -1;
		}
	} else {
		if (pcl::io::loadPLYFile(argv[filenames[0]], *source_cloud) < 0) {
			std::cout << "Error loading point cloud " << argv[filenames[0]]
					<< std::endl << std::endl;
			return -1;
		}
	}

	if (pcl::console::find_argument(argc, argv, "-m") >= 0) {
		bool setUnseenToMaxRange = true;
		std::cout
				<< "Setting unseen values in range image to maximum range readings.\n";
	}
	int tmp_coordinate_frame;
	if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0) {
		int coordinate_frame = pcl::RangeImage::CoordinateFrame(
				tmp_coordinate_frame);
		std::cout << "Using coordinate frame " << (int) coordinate_frame
				<< ".\n";
	}
	float angular_resolution = 0.5f;
	float support_size = 0.2f;
	pcl::RangeImage::CoordinateFrame coordinate_frame =
			pcl::RangeImage::CAMERA_FRAME;
	bool setUnseenToMaxRange = false;

	// --------------------------------
	// -----Extract NARF keypoints-----
	// --------------------------------
	pcl::RangeImageBorderExtractor range_image_border_extractor;
	pcl::NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);
	pcl::NarfKeypoint hola(&range_image_border_extractor);

	narf_keypoint_detector.getParameters().support_size = support_size;
	//narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
	//narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;

	pcl::PointCloud<int> keypoint_indices;
	narf_keypoint_detector.compute(keypoint_indices);
	std::cout << "Found " << keypoint_indices.points.size() << " key points.\n";

	return 0;
}
