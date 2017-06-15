/* \author Adrian Arroyo - adr.arroyo.perez@gmail.com */

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
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/spin_image.h>

typedef pcl::PointXYZ PointType;

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution = 0.5f;
float support_size = 0.2f;
pcl::RangeImage::CoordinateFrame coordinate_frame =
		pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;
bool rotation_invariant = true;

// --------------
// -----Help-----
// --------------
void printUsage(const char* progName) {
	std::cout << "\n\nUsage: " << progName << " [options] <scene.pcd>\n\n"
			<< "Options:\n" << "-------------------------------------------\n"
			<< "-r <float>   angular resolution in degrees (default "
			<< angular_resolution << ")\n"
			<< "-c <int>     coordinate frame (default "
			<< (int) coordinate_frame << ")\n"
			<< "-m           Treat all unseen points to max range\n"
			<< "-s <float>   support size for the interest points (diameter of the used sphere - "
					"default " << support_size << ")\n"
			<< "-o <0/1>     switch rotational invariant version of the feature on/off"
			<< " (default " << (int) rotation_invariant << ")\n"
			<< "-h           this help\n" << "\n\n";
}

void setViewerPose(pcl::visualization::PCLVisualizer& viewer,
		const Eigen::Affine3f& viewer_pose) {
	Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
	Eigen::Vector3f look_at_vector = viewer_pose.rotation()
			* Eigen::Vector3f(0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector = viewer_pose.rotation()
			* Eigen::Vector3f(0, -1, 0);
	viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
			look_at_vector[0], look_at_vector[1], look_at_vector[2],
			up_vector[0], up_vector[1], up_vector[2]);
}

// --------------
// -----Main-----
// --------------
int main(int argc, char** argv) {
	// --------------------------------------
	// -----Parse Command Line Arguments-----
	// --------------------------------------
	if (pcl::console::find_argument(argc, argv, "-h") >= 0) {
		printUsage(argv[0]);
		return 0;
	}
	if (pcl::console::find_argument(argc, argv, "-m") >= 0) {
		setUnseenToMaxRange = true;
		cout
				<< "Setting unseen values in range image to maximum range readings.\n";
	}
	if (pcl::console::parse(argc, argv, "-o", rotation_invariant) >= 0)
		cout << "Switching rotation invariant feature version "
				<< (rotation_invariant ? "on" : "off") << ".\n";
	int tmp_coordinate_frame;
	if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0) {
		coordinate_frame = pcl::RangeImage::CoordinateFrame(
				tmp_coordinate_frame);
		cout << "Using coordinate frame " << (int) coordinate_frame << ".\n";
	}
	if (pcl::console::parse(argc, argv, "-s", support_size) >= 0)
		cout << "Setting support size to " << support_size << ".\n";
	if (pcl::console::parse(argc, argv, "-r", angular_resolution) >= 0)
		cout << "Setting angular resolution to " << angular_resolution
				<< "deg.\n";
	angular_resolution = pcl::deg2rad(angular_resolution);

	// ------------------------------------------------------------------
	// -----Read ply file-----
	// ------------------------------------------------------------------
	pcl::PointCloud<PointType>::Ptr point_cloud_ptr(
			new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
	std::vector<int> pcd_filename_indices =
			pcl::console::parse_file_extension_argument(argc, argv, "ply");
	std::string filename = argv[pcd_filename_indices[0]];
	if (pcl::io::loadPLYFile(filename, point_cloud) == -1) {
		cerr << "Was not able to open file \"" << filename << "\".\n";
		printUsage(argv[0]);
		return 0;
	}
	scene_sensor_pose = Eigen::Affine3f(
			Eigen::Translation3f(point_cloud.sensor_origin_[0],
					point_cloud.sensor_origin_[1],
					point_cloud.sensor_origin_[2]))
			* Eigen::Affine3f(point_cloud.sensor_orientation_);

	//---------EXTRACTION OF NARF FEATURES------------
	// -----------------------------------------------
	// -----Create RangeImage from the PointCloud-----
	// -----------------------------------------------
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;
	range_image.createFromPointCloud(point_cloud, angular_resolution,
			pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), scene_sensor_pose,
			coordinate_frame, noise_level, min_range, border_size);
	range_image.integrateFarRanges(far_ranges);
	if (setUnseenToMaxRange)
		range_image.setUnseenToMaxRange();

	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(
			range_image_ptr, 0, 0, 0);
	viewer.addPointCloud(range_image_ptr, range_image_color_handler,
			"range image");
	viewer.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
	//viewer.addCoordinateSystem (1.0f, "global");
	//PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
	//viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
	viewer.initCameraParameters();
	setViewerPose(viewer, range_image.getTransformationToWorldSystem());

	// --------------------------------
	// -----Extract NARF keypoints-----
	// --------------------------------
	pcl::RangeImageBorderExtractor range_image_border_extractor;
	pcl::NarfKeypoint narf_keypoint_detector;
	narf_keypoint_detector.setRangeImageBorderExtractor(
			&range_image_border_extractor);
	narf_keypoint_detector.setRangeImage(&range_image);
	narf_keypoint_detector.getParameters().support_size = support_size;

	pcl::PointCloud<int> keypoint_indices;
	narf_keypoint_detector.compute(keypoint_indices);
	std::cout << "Found " << keypoint_indices.points.size() << " key points.\n";

	// -------------------------------------
	// -----Show keypoints in 3D viewer-----
	// -------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
	keypoints.points.resize(keypoint_indices.points.size());
	for (size_t i = 0; i < keypoint_indices.points.size(); ++i)
		keypoints.points[i].getVector3fMap() =
				range_image.points[keypoint_indices.points[i]].getVector3fMap();
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(
			keypoints_ptr, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ>(keypoints_ptr, keypoints_color_handler,
			"keypoints");
	viewer.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

	// ------------------------------------------------------
	// -----Extract NARF descriptors for interest points-----
	// ------------------------------------------------------
	std::vector<int> keypoint_indices2;
	keypoint_indices2.resize(keypoint_indices.points.size());
	for (unsigned int i = 0; i < keypoint_indices.size(); ++i) // This step is necessary to get the right vector type
		keypoint_indices2[i] = keypoint_indices.points[i];
	pcl::NarfDescriptor narf_descriptor(&range_image, &keypoint_indices2);
	narf_descriptor.getParameters().support_size = support_size;
	narf_descriptor.getParameters().rotation_invariant = rotation_invariant;
	pcl::PointCloud<pcl::Narf36> narf_descriptors;
	narf_descriptor.compute(narf_descriptors);
	cout << "Extracted " << narf_descriptors.size() << " descriptors for "
			<< keypoint_indices.points.size() << " keypoints.\n";

	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
		pcl_sleep(0.01);
	}

	// ------------------------------------------------------
	// ---------------Extract PFH features-------------------
	// ------------------------------------------------------
	// Normal estimation*
	/*pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	 pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	 pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
	 new pcl::search::KdTree<pcl::PointXYZ>);
	 tree->setInputCloud(point_cloud_ptr);
	 n.setInputCloud(point_cloud_ptr);
	 n.setSearchMethod(tree);
	 n.setKSearch(20);
	 n.compute(*normals);

	 // Create the PFH estimation class, and pass the input dataset+normals to it
	 /*pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	 pfh.setInputCloud(point_cloud_ptr);
	 pfh.setInputNormals(normals);

	 // Create an empty kdtree representation, and pass it to the PFH estimation object.
	 // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	 pfh.setSearchMethod(tree);

	 // Output datasets
	 pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(
	 new pcl::PointCloud<pcl::PFHSignature125>());

	 // Use all neighbors in a sphere of radius 5cm
	 // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	 pfh.setRadiusSearch(0.05);

	 // Check for NaNs in normals
	 for (int i = 0; i < normals->points.size(); i++) {
	 if (!pcl::isFinite<pcl::Normal>(normals->points[i])) {
	 PCL_WARN("normals[%d] is not finite\n", i);
	 }
	 }

	 // Compute the features
	 pfh.compute(*pfhs);

	 cout << "Extracted " << pfh.k_ << "PFH features";*/

	// ------------------------------------------------------
	// ------------Extract Spin Image Estimator--------------
	// ------------------------------------------------------
	// Setup the spin images computation
	/*typedef pcl::Histogram<153> SpinImage;
	pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> spinImageEstimation(
			8, 0.5, 16);
	spinImageEstimation.setInputCloud(point_cloud_ptr);
	spinImageEstimation.setInputNormals(normals);

	// Use the same KdTree from the normal estimation
	spinImageEstimation.setSearchMethod(tree);
	pcl::PointCloud<SpinImage>::Ptr spinImages(new pcl::PointCloud<SpinImage>);
	spinImageEstimation.setRadiusSearch (2);
	//spinImageEstimation.setKSearch(10);

	// Actually compute the spin images
	spinImageEstimation.compute(*spinImages);
	std::cout << "SI output points.size (): " << spinImages->points.size()
			<< std::endl;

	// Display and retrieve the spin image descriptor vector for the first point.
	SpinImage descriptor = spinImages->points[0];
	std::cout << descriptor << std::endl;*/

	return 1;
}
