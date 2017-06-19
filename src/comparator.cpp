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
#include <pcl/surface/gp3.h>
#include <pcl/features/spin_image.h>
#include <cmath>
#include <pcl/registration/icp.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <vector>

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

void processNARF (std::string filename, std::string filename2){
// ------------------------------------------------------------------
	// -----Read ply file-----
	// ------------------------------------------------------------------
	pcl::PointCloud<PointType>::Ptr point_cloud_ptr(
			new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
	if (pcl::io::loadPLYFile(filename, point_cloud) == -1) {
		cerr << "Was not able to open file \"" << filename << "\".\n";
		printUsage("");
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
	/*pcl::visualization::PCLVisualizer viewer("3D Viewer");
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
	setViewerPose(viewer, range_image.getTransformationToWorldSystem());*/

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

	// -------------------------------------
	// -----Show keypoints in 3D viewer-----
	// -------------------------------------
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(
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
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");*/

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
	
	
	//-----------------------------------------------------
	//-----------------SECOND PCL--------------------------
	//-----------------------------------------------------
	pcl::PointCloud<PointType>::Ptr point_cloud_ptr2(
			new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>& point_cloud2 = *point_cloud_ptr2;
	
	if (pcl::io::loadPLYFile(filename2, point_cloud2) == -1) {
		cerr << "Was not able to open file \"" << filename2 << "\".\n";
		printUsage("");
	}
	/*point_cloud2=point_cloud;
	point_cloud2.resize(point_cloud.size()/2);*/
	scene_sensor_pose = Eigen::Affine3f(
			Eigen::Translation3f(point_cloud2.sensor_origin_[0],
					point_cloud.sensor_origin_[1],
					point_cloud.sensor_origin_[2]))
			* Eigen::Affine3f(point_cloud.sensor_orientation_);

	//---------EXTRACTION OF NARF FEATURES------------
	// -----------------------------------------------
	// -----Create RangeImage from the PointCloud-----
	// -----------------------------------------------
	boost::shared_ptr<pcl::RangeImage> range_image2_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image2 = *range_image2_ptr;
	range_image2.createFromPointCloud(point_cloud2, angular_resolution,
			pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), scene_sensor_pose,
			coordinate_frame, noise_level, min_range, border_size);
	range_image2.integrateFarRanges(far_ranges);
	if (setUnseenToMaxRange)
		range_image2.setUnseenToMaxRange();

	// --------------------------------
	// -----Extract NARF keypoints-----
	// --------------------------------
	pcl::RangeImageBorderExtractor range_image_border_extractor2;
	pcl::NarfKeypoint narf_keypoint_detector2;
	narf_keypoint_detector2.setRangeImageBorderExtractor(
			&range_image_border_extractor2);
	narf_keypoint_detector2.setRangeImage(&range_image2);
	narf_keypoint_detector2.getParameters().support_size = support_size;
	pcl::PointCloud<int> keypoint_indices3;
	narf_keypoint_detector2.compute(keypoint_indices3);
	
	// ------------------------------------------------------
	// -----Extract NARF descriptors for interest points-----
	// ------------------------------------------------------
	std::vector<int> keypoint_indices4;
	keypoint_indices4.resize(keypoint_indices3.points.size());
	for (unsigned int i = 0; i < keypoint_indices3.size(); ++i) // This step is necessary to get the right vector type
		keypoint_indices4[i] = keypoint_indices3.points[i];
	pcl::NarfDescriptor narf_descriptor2(&range_image2, &keypoint_indices4);
	narf_descriptor2.getParameters().support_size = support_size;
	narf_descriptor2.getParameters().rotation_invariant = rotation_invariant;
	pcl::PointCloud<pcl::Narf36> narf_descriptors2;
	narf_descriptor2.compute(narf_descriptors2);
	

	cout << "Primer pcl:\n";
	std::cout << "Found " << keypoint_indices.points.size() << " key points.\n";
	cout << "Extracted " << narf_descriptors.size() << " descriptors for "
			<< keypoint_indices.points.size() << " keypoints.\n";

	cout << "Segundo pcl:\n";
	std::cout << "Found " << keypoint_indices3.points.size() << " key points.\n";
	cout << "Extracted " << narf_descriptors2.size() << " descriptors for "
			<< keypoint_indices3.points.size() << " keypoints.\n";

	//Comparacion truño
	/*int j=0;
	pcl::Narf36 feat1;
	pcl::Narf36 feat2;
	for (int i=0; i<narf_descriptors.points.size(); ++i){
		feat1=narf_descriptors.points[i];
		//cout << feat1.x << " " << feat1.y << " " << feat1.z << "\n ";
		for (int k=0; k<narf_descriptors2.points.size(); ++k){
			feat2=narf_descriptors2.points[k];
			//if(feat1.x == feat2.x && feat1.y == feat2.y && feat1.z == feat2.z){
			//if(feat1.descriptor == feat2.descriptor){
			if(sqrt(pow(feat1.x-feat2.x,2)+pow(feat1.y-feat2.y,2)+pow(feat1.z-feat2.z,2))<0.5 && 
				abs(feat1.roll-feat2.roll)<0.05 && abs(feat1.pitch-feat2.pitch)<0.05 && abs(feat1.yaw-feat2.yaw)<0.05){
				//cout << "narf IGUALES\n\n";
				++j;
				//cout << feat1.x << " " << feat1.y << " " << feat1.z << " \n";
				//cout << feat2.x << " " << feat2.y << " " << feat2.z << " \n";
				/*for(int t=0; t<feat1.descriptorSize(); ++t){
					cout << feat1.descriptor[t] << ";";
				}
				cout << "] \n";
				for(int t=0; t<feat2.descriptorSize(); ++t){
					cout << feat2.descriptor[t] << ";";
				}
				cout << "] \n\n";
				//break;
			}
		}
	}
	cout << "Found " << j << " same narf descriptors between PCLs\n";*/
	//--------------------
	// -----Main loop-----
	//--------------------
	/*while (!viewer.wasStopped()) {
		viewer.spinOnce();
		pcl_sleep(0.01);
	}*/

	// A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
	pcl::KdTreeFLANN<pcl::Narf36> matching = new pcl::KdTreeFLANN<pcl::Narf36>(false);
	pcl::KdTree<pcl::Narf36>::PointCloudConstPtr ptr_narf_descriptors(&narf_descriptors);
	matching.setInputCloud(ptr_narf_descriptors);
	// A Correspondence object stores the indices of the query and the match,
	// and the distance/weight.
	std::vector<int> correspondence;

	// Check every descriptor computed for the scene.
	for (size_t i = 0; i < narf_descriptors2.size(); ++i)
	{
		std::vector<int> neighbors(1);
		std::vector<float> squaredDistances(1);
		// Ignore NaNs.
		if (pcl_isfinite(narf_descriptors2.at(i).descriptor[0]))
		{
			// Find the nearest neighbor (in descriptor space)...
			int neighborCount = matching.nearestKSearch(narf_descriptors2.at(i), 1, neighbors, squaredDistances);
			// ...and add a new correspondence if the distance is less than a threshold
			// (SHOT distances are between 0 and 1, other descriptors use different metrics).
			if (neighborCount == 1 && squaredDistances[0] < 0.25f)
			{
				//correspondence.push_back(neighbors[0], static_cast<int>(i), squaredDistances[0]);
				correspondence.push_back(neighbors[0]);
			}
		}
	}
	std::cout << "Found " << correspondence.size() << " correspondences\n";
}

/*void processSift (std::string filename, std::string filename2){
	// ------------------------------------------------------------------
	// -----Read ply file-----
	// ------------------------------------------------------------------
	pcl::PointCloud<PointType>::Ptr cloud_xyz_ptr(
			new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>& cloud_xyz = *cloud_xyz_ptr;
	if (pcl::io::loadPLYFile(filename, cloud_xyz) == -1) {
		cerr << "Was not able to open file \"" << filename << "\".\n";
		printUsage("");
	}

	// Parameters for sift computation
	const float min_scale = 0.01f;
	const int n_octaves = 3;
	const int n_scales_per_octave = 4;
	const float min_contrast = 0.001f;

	// Estimate the sift interest points using normals values from xyz as the Intensity variants
	pcl::SIFTKeypoint<PointType, pcl::PointWithScale> sift;
	pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType> ());;//new API
	pcl::PointCloud<pcl::PointWithScale>::Ptr sifts (new pcl::PointCloud<pcl::PointWithScale>);
	tree.setInputCloud(cloud_xyz_ptr, NULL);
	sift.setSearchMethod (tree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.compute (*sifts);

	cout << "Computed " << sifts.points.size () << " SIFT Keypoints";

}*/

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

	std::vector<int> pcl_filename_indices =
			pcl::console::parse_file_extension_argument(argc, argv, "ply");

	processNARF(argv[pcl_filename_indices[0]], argv[pcl_filename_indices[1]]);


	//processSift(argv[pcl_filename_indices[0]], argv[pcl_filename_indices[1]]);

	return 1;
}
