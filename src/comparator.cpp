/*
 * comparator.cpp
 *
 *      Author: Adrian Arroyo - adr.arroyo.perez@gmail.com

 */
#include "../include/comparator.h"

typedef pcl::Histogram<32> RIFT32;
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
bool seeClusters = false;
bool noise = false;
bool euclidean = false;
bool icp = false;

// --------------
// -----Help-----
// --------------
void printUsage() {
	std::cout
			<< "\n\nUsage: [options] </pathToScene1.ply> </pathToScene2.ply>\n\n"
			<< "Options:\n" << "-------------------------------------------\n"
			<< "-n activate noise analysis \n"
			<< "-v activate visualization of clusters and matches\n"
			<< "-i activate ICP algorithm to know if both point clouds are enough similar \n"
			<< "-e activate euclidean cluster segmentation as main segmentation algorithm; region growing segmentation is default\n"
			<< "-h show this help\n" << "\n\n";
}

/*void setViewerPose(pcl::visualization::PCLVisualizer& viewer,
 const Eigen::Affine3f& viewer_pose) {
 Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
 Eigen::Vector3f look_at_vector = viewer_pose.rotation()
 * Eigen::Vector3f(0, 0, 1) + pos_vector;
 Eigen::Vector3f up_vector = viewer_pose.rotation()
 * Eigen::Vector3f(0, -1, 0);
 viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
 look_at_vector[0], look_at_vector[1], look_at_vector[2],
 up_vector[0], up_vector[1], up_vector[2]);
 }*/

/*void spatial_change_detection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA,
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB) {*/
void spatial_change_detection(std::string filename, std::string filename2) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPLYFile(filename, *cloudA) == -1) {
		cerr << "Was not able to open file \"" << filename << "\".\n";
		printUsage();
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPLYFile(filename2, *cloudB) == -1) {
		cerr << "Was not able to open file \"" << filename2 << "\".\n";
		printUsage();
	}

	srand((unsigned int) time(NULL));

	// Octree resolution - side length of octree voxels
	float resolution = 32.0f;

	// Instantiate octree-based point cloud change detection class
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree(
			resolution);

	// Add points from cloudA to octree
	octree.setInputCloud(cloudA);
	octree.addPointsFromInputCloud();

	// Switch octree buffers: This resets octree but keeps previous tree structure in memory.
	octree.switchBuffers();

	// Add points from cloudB to octree
	octree.setInputCloud(cloudB);
	octree.addPointsFromInputCloud();

	std::vector<int> newPointIdxVector;

	// Get vector of point indices from octree voxels which did not exist in previous buffer
	octree.getPointIndicesFromNewVoxels(newPointIdxVector);

	// Output points
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDiff(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	for (size_t i = 0; i < newPointIdxVector.size(); ++i) {
		pcl::PointXYZRGB point;
		//cloudDiff->points[i] = cloudB->points[newPointIdxVector[i]];
		point.x = cloudB->points[newPointIdxVector[i]].x;
		point.y = cloudB->points[newPointIdxVector[i]].y;
		point.z = cloudB->points[newPointIdxVector[i]].z;
		point.rgb = cloudB->points[newPointIdxVector[i]].rgb;
		cloudDiff->push_back(point);
	}
	pcl::visualization::CloudViewer viewer("Cluster viewer");
	viewer.showCloud(cloudDiff);
	while (!viewer.wasStopped()) {
	}
}

void processNARF2(std::string filename, std::string filename2) {
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
		printUsage();
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
	for (unsigned int i = 0; i < keypoint_indices.size(); ++i)// This step is necessary to get the right vector type
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
		printUsage();
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
	for (unsigned int i = 0; i < keypoint_indices3.size(); ++i)	// This step is necessary to get the right vector type
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
	std::cout << "Found " << keypoint_indices3.points.size()
			<< " key points.\n";
	cout << "Extracted " << narf_descriptors2.size() << " descriptors for "
			<< keypoint_indices3.points.size() << " keypoints.\n";

	//Comparacion basica
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
	pcl::KdTreeFLANN<pcl::Narf36> matching = new pcl::KdTreeFLANN<pcl::Narf36>(
			false);
	pcl::KdTree<pcl::Narf36>::PointCloudConstPtr ptr_narf_descriptors(
			&narf_descriptors);
	matching.setInputCloud(ptr_narf_descriptors);
	// A Correspondence object stores the indices of the query and the match,
	// and the distance/weight.
	std::vector<int> correspondence;
	// Check every descriptor computed for the scene.
	for (size_t i = 0; i < narf_descriptors2.points.size(); ++i) {
		std::vector<int> neighbors(1);
		std::vector<float> squaredDistances(1);
		// Ignore NaNs.
		if (pcl_isfinite(narf_descriptors2.at(i).descriptor[0])) {
			// Find the nearest neighbor (in descriptor space)...
			int neighborCount = matching.nearestKSearch(narf_descriptors2.at(i),
					1, neighbors, squaredDistances);
			// ...and add a new correspondence if the distance is less than a threshold
			// (SHOT distances are between 0 and 1, other descriptors use different metrics).
			if (neighborCount == 1 && squaredDistances[0] < 0.25f) {
				//correspondence.push_back(neighbors[0], static_cast<int>(i), squaredDistances[0]);
				correspondence.push_back(neighbors[0]);
			}
		}
	}
	std::cout << "Found " << correspondence.size() << " correspondences\n";
}

int matchNarf(pcl::PointCloud<pcl::Narf36>::Ptr narf_descriptors,
		pcl::PointCloud<pcl::Narf36>::Ptr narf_descriptors2) {
	// A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
	pcl::KdTreeFLANN<pcl::Narf36> matching = new pcl::KdTreeFLANN<pcl::Narf36>(
			false);
	pcl::KdTree<pcl::Narf36>::PointCloudConstPtr ptr_narf_descriptors(
			narf_descriptors);
	matching.setInputCloud(narf_descriptors);
	// A Correspondence object stores the indices of the query and the match,
	// and the distance/weight.
	std::vector<int> correspondence(1);

	// Check every descriptor computed for the scene.
	for (int i = 0; i < narf_descriptors2->points.size(); ++i) {
		std::vector<int> neighbors(1);
		std::vector<float> squaredDistances(1);
		// Ignore NaNs.
		if (pcl_isfinite(narf_descriptors2->at(i).descriptor[0])) {
			// Find the nearest neighbor (in descriptor space)...
			int neighborCount = matching.nearestKSearch(
					narf_descriptors2->at(i), 1, neighbors, squaredDistances);
			// ...and add a new correspondence if the distance is less than a threshold
			// (SHOT distances are between 0 and 1, other descriptors use different metrics).
			if (neighborCount == 1 && squaredDistances[0] < 0.25f) {
				//correspondence.push_back(neighbors[0], static_cast<int>(i), squaredDistances[0]);
				correspondence.push_back(0);
			}
		}
	}
	//std::cout << "Found " << correspondence.size() << " correspondences\n";
	return correspondence.size();
}

pcl::PointCloud<pcl::Narf36>::Ptr processNARF(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr) {
	pcl::PointCloud<pcl::PointXYZRGB>& point_cloud = *point_cloud_ptr;
	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
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

	// ------------------------------------------------------
	// -----Extract NARF descriptors for interest points-----
	// ------------------------------------------------------
	std::vector<int> keypoint_indices2;
	keypoint_indices2.resize(keypoint_indices.points.size());
	for (unsigned int i = 0; i < keypoint_indices.size(); ++i)// This step is necessary to get the right vector type
		keypoint_indices2[i] = keypoint_indices.points[i];
	pcl::NarfDescriptor narf_descriptor(&range_image, &keypoint_indices2);
	narf_descriptor.getParameters().support_size = support_size;
	narf_descriptor.getParameters().rotation_invariant = rotation_invariant;
	pcl::PointCloud<pcl::Narf36>::Ptr narf_descriptors;
	narf_descriptor.compute(*narf_descriptors);

	std::cout << "Computed " << narf_descriptors->points.size()
			<< " NARF descriptors" << std::endl;
	return (narf_descriptors);
}

/*pcl::PointCloud<pcl::PointWithScale> processSift(std::string filename,
 std::string filename2) {*/
pcl::PointCloud<pcl::PointWithScale> processSift(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz_ptr) {
	// ------------------------------------------------------------------
	// -----Read ply file-----
	// ------------------------------------------------------------------
	/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz_ptr(
	 new pcl::PointCloud<pcl::PointXYZRGB>);
	 pcl::PointCloud<pcl::PointXYZRGB>& cloud_xyz = *cloud_xyz_ptr;
	 if (pcl::io::loadPLYFile(filename, cloud_xyz) == -1) {
	 cerr << "Was not able to open file \"" << filename << "\".\n";
	 printUsage();
	 }*/

	// Parameters for sift computation
	const float min_scale = 0.01f;
	const int n_octaves = 3;
	const int n_scales_per_octave = 4;
	const float min_contrast = 0.001f;

	// Estimate the sift interest points using normals values from xyz as the Intensity variants
	pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
	pcl::search::KdTree<pcl::PointXYZRGB> tree = new pcl::search::KdTree<
			pcl::PointXYZRGB>();
	pcl::PointCloud<pcl::PointWithScale> sifts;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(
			new pcl::search::KdTree<pcl::PointXYZRGB>);
	sift.setInputCloud(cloud_xyz_ptr);
	sift.setSearchMethod(kdtree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.compute(sifts);

	cout << "Computed " << sifts.points.size() << " SIFT Keypoints\n";
	return sifts;
}

double matchVHF(pcl::PointCloud<pcl::VFHSignature308>::Ptr vhf1,
		pcl::PointCloud<pcl::VFHSignature308>::Ptr vhf2) {
	double dis = 0;
	for (int i = 0; i < vhf1->at(0).descriptorSize(); ++i) {
		dis += pow(vhf1->at(0).histogram[i] - vhf2->at(0).histogram[i], 2);
		//std::cout << "des1 position: " << p1.histogram[i] << std::endl;
	}
	dis = sqrt(dis);
	return dis;
}

pcl::PointCloud<pcl::VFHSignature308>::Ptr processVHF(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the VFH descriptor.
	pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(
			new pcl::PointCloud<pcl::VFHSignature308>);

	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(
			new pcl::search::KdTree<pcl::PointXYZRGB>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	// VFH estimation object.
	pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud(cloud);
	vfh.setInputNormals(normals);
	vfh.setSearchMethod(kdtree);
	// Optionally, we can normalize the bins of the resulting histogram,
	// using the total number of points.
	vfh.setNormalizeBins(true);
	// Also, we can normalize the SDC with the maximum size found between
	// the centroid and any of the cluster's points.
	vfh.setNormalizeDistance(false);

	vfh.compute(*descriptor);

	//cout << "Computed " << descriptor->points.size() << " VHF descriptor\n";
	return descriptor;
}

/**
 * Distance between two rift descriptors
 */
double distanceRIFT(RIFT32 p1, RIFT32 p2) {
	double dis = 0;
	for (int i = 0; i < 32; ++i) {
		dis += pow(p1.histogram[i] - p2.histogram[i], 2);
		/*if (!pcl_isfinite(p1.histogram[i])) {
		 /*std::cout << "des1 is not finite " << std::endl;
		 for (int j = 0; j < 32; ++j) {
		 std::cout << ", " << p1.histogram[j];
		 }
		 std::cout << std::endl;
		 }*/
	}
	dis = sqrt(dis);
	return dis;
}

int matchRIFTFeatures(pcl::PointCloud<RIFT32>::Ptr descriptors1,
		pcl::PointCloud<RIFT32>::Ptr descriptors2) {
	int correspondences = 0;
	for (int i = 0; i < descriptors1->points.size(); ++i) {
		RIFT32 des1 = descriptors1->points[i];
		double minDis = 100000000000000000;
		double actDis = 0;
		//Find closest descriptor
		for (int j = 0; j < descriptors2->points.size(); ++j) {
			actDis = distanceRIFT(des1, descriptors2->points[j]);
			//std::cout << "act distance: " << actDis << std::endl;
			if (actDis < minDis) {
				minDis = actDis;
			}
		}
		//std::cout << "min distance: " << minDis << std::endl;
		//If distance between both descriptors is less than threshold we found correspondence
		if (minDis < 0.5)
			++correspondences;
	}
	return correspondences;
}

std::vector<int> matchRIFTFeaturesKnn(pcl::PointCloud<RIFT32>::Ptr descriptors1,
		pcl::PointCloud<RIFT32>::Ptr descriptors2) {

	// A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
	pcl::KdTreeFLANN<RIFT32> matching = new pcl::KdTreeFLANN<RIFT32>(false);
	matching.setInputCloud(descriptors1);
	// A Correspondence object stores the indices of the query and the match,
	// and the distance/weight.
	std::vector<int> correspondence(1);

	// Check every descriptor computed for the scene.
	for (size_t i = 0; i < descriptors2->points.size(); ++i) {
		std::vector<int> neighbors(1);
		std::vector<float> squaredDistances(1);
		//if (pcl_isfinite(descriptors2->at(i).histogram[0])) {
		// Find the nearest neighbor (in descriptor space)...
		int neighborCount = matching.nearestKSearch(descriptors2->at(i), 1,
				neighbors, squaredDistances);
		// ...and add a new correspondence if the distance is less than a threshold
		if (neighborCount == 1 && squaredDistances[0] < 0.05f) {
			correspondence.push_back(neighbors[0]);
		}
		//} else
		//std::cout << "Found NaN in RIFT descriptors" << std::endl;
	}
	//std::cout << "Found " << correspondence.size() << " correspondences\n";

	return correspondence;
}

pcl::PointCloud<RIFT32>::Ptr processRIFT(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

	// Object for storing the point cloud with intensity value.
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIntensityGlobal(
			new pcl::PointCloud<pcl::PointXYZI>);
	// Convert the RGB to intensity.
	pcl::PointCloudXYZRGBtoXYZI(*cloud, *cloudIntensityGlobal);

	/*
	 pcl::visualization::CloudViewer viewer("Cluster viewer");
	 viewer.showCloud(cloud_Color_test);
	 while (!viewer.wasStopped()) {
	 }

	 pcl::visualization::CloudViewer viewer2("Cluster viewer");
	 viewer2.showCloud(cloud_Color);
	 while (!viewer2.wasStopped()) {
	 }*/

	// Object for storing the intensity gradients.
	pcl::PointCloud<pcl::IntensityGradient>::Ptr gradients(
			new pcl::PointCloud<pcl::IntensityGradient>);
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the RIFT descriptor for each point.
	pcl::PointCloud<RIFT32>::Ptr descriptors(new pcl::PointCloud<RIFT32>());

	/*pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
	 new pcl::PointCloud<pcl::PointXYZRGB>);
	 vg.setInputCloud(cloudColor);
	 vg.setLeafSize(0.01f, 0.01f, 0.01f);
	 vg.filter(*cloud_filtered);
	 std::cout << "PointCloud after filtering has: "
	 << cloud_filtered->points.size() << " data points." << std::endl;*/

	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloudIntensityGlobal);
	//normalEstimation.setSearchSurface(cloudIntensityGlobal);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(
			new pcl::search::KdTree<pcl::PointXYZI>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	std::vector<int> indices;
	pcl::removeNaNNormalsFromPointCloud(*normals, *normals, indices);
	std::cout << "Point cloud normals size after removing NaNs: "
			<< normals->points.size() << std::endl;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIntensityGlobal2(
			new pcl::PointCloud<pcl::PointXYZI>);
	for (int i = 0; i < indices.size(); ++i) {
		cloudIntensityGlobal2->push_back(
				cloudIntensityGlobal->points[indices[i]]);
	}

	// Compute the intensity gradients.
	pcl::IntensityGradientEstimation<pcl::PointXYZI, pcl::Normal,
			pcl::IntensityGradient,
			pcl::common::IntensityFieldAccessor<pcl::PointXYZI> > ge;
	ge.setInputCloud(cloudIntensityGlobal2);
	//ge.setSearchSurface(cloudIntensityGlobal);
	ge.setInputNormals(normals);
	ge.setRadiusSearch(0.03);
	ge.compute(*gradients);

	// RIFT estimation object.
	pcl::RIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, RIFT32> rift;
	rift.setInputCloud(cloudIntensityGlobal2);
	//rift.setSearchSurface(cloudIntensityGlobal);
	rift.setSearchMethod(kdtree);
	// Set the intensity gradients to use.
	rift.setInputGradient(gradients);
	// Radius, to get all neighbors within.
	rift.setRadiusSearch(0.05);
	// Set the number of bins to use in the distance dimension.
	rift.setNrDistanceBins(4);
	// Set the number of bins to use in the gradient orientation dimension.
	rift.setNrGradientBins(8);
	// Note: you must change the output histogram size to reflect the previous values.

	rift.compute(*descriptors);
	pcl::PointCloud<RIFT32>::Ptr descriptors2(new pcl::PointCloud<RIFT32>());
	for (int i = 0; i < descriptors->points.size(); ++i) {
		RIFT32 des = descriptors->points[i];
		if (pcl_isfinite(des.histogram[0])) {
			descriptors2->push_back(des);
			//std::cout << "RIFT value: " << des.histogram[0] << std::endl;
		}
	}
	cout << "Computed " << descriptors2->points.size() << " RIFT descriptors\n";
	return descriptors2;
}

pcl::PointCloud<RIFT32>::Ptr processRIFTwithSIFT(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

	//We find the sift interesting keypoints in the pointclouds
	pcl::PointCloud<pcl::PointWithScale> sifts = processSift(cloud);
	//We find the corresponding point of the sift keypoint in the original
	//cloud and store it with RGB so that it can be transformed into intensity
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Color(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>& point_cloud_sift = *cloud_Color;
	for (int i = 0; i < sifts.points.size(); ++i) {
		pcl::PointWithScale pointSift = sifts.points[i];
		pcl::PointXYZRGB point;
		for (int j = 0; j < cloud->points.size(); ++j) {
			point = cloud->points[j];
			//TODO comparacion un poco basica - por alguna razon los keypoints de sift no tienen la misma coordenada q los puntos de la nube original
			if (sqrt(
					pow(pointSift.x - point.x, 2)
							+ pow(pointSift.y - point.y, 2)
							+ pow(pointSift.z - point.z, 2)) < 0.05) {
				point_cloud_sift.push_back(point);
				/*std::cout << point_cloud_sift.back().x << " "
				 << point_cloud_sift.back().y << " "
				 << point_cloud_sift.back().z << std::endl;*/
				break;
			}
		}
	}

	/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Color_test(
	 new pcl::PointCloud<pcl::PointXYZRGB>);
	 pcl::PointCloud<pcl::PointXYZRGB>& point_cloud_sift_test = *cloud_Color_test;
	 for (int i = 0; i < sifts.points.size(); ++i) {
	 pcl::PointXYZRGB point;
	 point.x = sifts.points[i].x;
	 point.y = sifts.points[i].y;
	 point.z = sifts.points[i].z;
	 point_cloud_sift_test.push_back(point);
	 }

	 pcl::visualization::CloudViewer viewer("Cluster viewer");
	 viewer.showCloud(cloud_Color_test);
	 while (!viewer.wasStopped()) {
	 }

	 pcl::visualization::CloudViewer viewer2("Cluster viewer");
	 viewer2.showCloud(cloud_Color);
	 while (!viewer2.wasStopped()) {
	 }*/

	/*cout << "Keypoint cloud has " << point_cloud_sift.points.size()
	 << " points\n";*/

	// Object for storing the point cloud with intensity value.
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIntensityKeypoints(
			new pcl::PointCloud<pcl::PointXYZI>);
	// Object for storing the intensity gradients.
	pcl::PointCloud<pcl::IntensityGradient>::Ptr gradients(
			new pcl::PointCloud<pcl::IntensityGradient>);
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the RIFT descriptor for each point.
	pcl::PointCloud<RIFT32>::Ptr descriptors(new pcl::PointCloud<RIFT32>());

	/*pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
	 new pcl::PointCloud<pcl::PointXYZRGB>);
	 vg.setInputCloud(cloudColor);
	 vg.setLeafSize(0.01f, 0.01f, 0.01f);
	 vg.filter(*cloud_filtered);
	 std::cout << "PointCloud after filtering has: "
	 << cloud_filtered->points.size() << " data points." << std::endl;*/

	// Convert the RGB to intensity.
	pcl::PointCloudXYZRGBtoXYZI(*cloud_Color, *cloudIntensityKeypoints);
	std::cout << "Size: " << cloudIntensityKeypoints->points.size() << "\n";

	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloudIntensityKeypoints);
	//normalEstimation.setSearchSurface(cloudIntensityGlobal);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(
			new pcl::search::KdTree<pcl::PointXYZI>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	std::vector<int> indices;
	pcl::removeNaNNormalsFromPointCloud(*normals, *normals, indices);
	std::cout << "Point cloud normals size after removing NaNs: "
			<< normals->points.size() << std::endl;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIntensityGlobal2(
			new pcl::PointCloud<pcl::PointXYZI>);
	for (int i = 0; i < indices.size(); ++i) {
		cloudIntensityGlobal2->push_back(
				cloudIntensityKeypoints->points[indices[i]]);
		/*cloudIntensityGlobal2->push_back(
		 cloudIntensityGlobal->points[indices[i]]);*/
	}

	// Compute the intensity gradients.
	pcl::IntensityGradientEstimation<pcl::PointXYZI, pcl::Normal,
			pcl::IntensityGradient,
			pcl::common::IntensityFieldAccessor<pcl::PointXYZI> > ge;
	ge.setInputCloud(cloudIntensityGlobal2);
	//ge.setSearchSurface(cloudIntensityGlobal);
	ge.setInputNormals(normals);
	ge.setRadiusSearch(0.03);
	ge.compute(*gradients);

	// RIFT estimation object.
	pcl::RIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, RIFT32> rift;
	rift.setInputCloud(cloudIntensityGlobal2);
	//rift.setSearchSurface(cloudIntensityGlobal);
	rift.setSearchMethod(kdtree);
	// Set the intensity gradients to use.
	rift.setInputGradient(gradients);
	// Radius, to get all neighbors within.
	rift.setRadiusSearch(0.05);
	// Set the number of bins to use in the distance dimension.
	rift.setNrDistanceBins(4);
	// Set the number of bins to use in the gradient orientation dimension.
	rift.setNrGradientBins(8);
	// Note: you must change the output histogram size to reflect the previous values.

	rift.compute(*descriptors);
	pcl::PointCloud<RIFT32>::Ptr descriptors2(new pcl::PointCloud<RIFT32>());
	for (int i = 0; i < descriptors->points.size(); ++i) {
		RIFT32 des = descriptors->points[i];
		if (pcl_isfinite(des.histogram[0])) {
			descriptors2->push_back(des);
			//std::cout << "RIFT value: " << des.histogram[0] << std::endl;
		}
	}
	cout << "Computed " << descriptors2->points.size() << " RIFT descriptors\n";
	return descriptors2;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr processFPFH(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudColor) {
	pcl::PointCloud<pcl::PointXYZRGB>& point_cloud = *cloudColor;
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the FPFH descriptors for each point.
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(
			new pcl::PointCloud<pcl::FPFHSignature33>());

	/*pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
	 new pcl::PointCloud<pcl::PointXYZRGB>);
	 vg.setInputCloud(cloudColor);
	 vg.setLeafSize(0.01f, 0.01f, 0.01f);
	 vg.filter(*cloud_filtered);
	 std::cout << "PointCloud after filtering has: "
	 << cloud_filtered->points.size() << " data points." << std::endl;*/

	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloudColor);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(
			new pcl::search::KdTree<pcl::PointXYZRGB>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);
	std::vector<int> indices2;
	pcl::removeNaNNormalsFromPointCloud(*normals, *normals, indices2);
	std::cout << "Point cloud normals size after removing NaNs: "
			<< normals->points.size() << std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudColorFinal(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < indices2.size(); ++i) {
		cloudColorFinal->push_back(cloudColor->points[indices2[i]]);
	}

	// FPFH estimation object.
	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud(cloudColorFinal);
	fpfh.setInputNormals(normals);
	fpfh.setSearchMethod(kdtree);
	// Search radius, to look for neighbors. Note: the value given here has to be
	// larger than the radius used to estimate the normals.
	fpfh.setRadiusSearch(0.05);

	fpfh.compute(*descriptors);

	std::cout << "Computed " << descriptors->points.size()
			<< " FPFH descriptors" << std::endl;
	return descriptors;
}

std::vector<int> matchFPFH(
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors1,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors2) {

	// A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
	pcl::KdTreeFLANN<pcl::FPFHSignature33> matching = new pcl::KdTreeFLANN<
			pcl::FPFHSignature33>(false);
	pcl::KdTree<pcl::FPFHSignature33>::PointCloudConstPtr ptr_descriptors1(
			descriptors1);
	matching.setInputCloud(descriptors1);
	// A Correspondence object stores the indices of the query and the match,
	// and the distance/weight.
	std::vector<int> correspondence(1);

	// Check every descriptor computed for the scene.
	for (size_t i = 0; i < descriptors2->points.size(); ++i) {
		std::vector<int> neighbors(1);
		std::vector<float> squaredDistances(1);
		if (pcl_isfinite(descriptors2->at(i).histogram[0])) {
			// Find the nearest neighbor (in descriptor space)...
			int neighborCount = matching.nearestKSearch(descriptors2->at(i), 1,
					neighbors, squaredDistances);
			// ...and add a new correspondence if the distance is less than a threshold
			// (SHOT distances are between 0 and 1, other descriptors use different metrics).
			if (neighborCount == 1 && squaredDistances[0] < 0.25f) {
				//correspondence.push_back(neighbors[0], static_cast<int>(i), squaredDistances[0]);
				correspondence.push_back(neighbors[0]);
			}
		} else
			std::cout << "Found NaN in FPFH descriptors" << std::endl;
	}
	//std::cout << "Found " << correspondence.size() << " correspondences\n";
	return correspondence;
}

pcl::PointCloud<pcl::SHOT352>::Ptr processSHOT(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

	pcl::PointCloud<pcl::PointWithScale> sifts = processSift(cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Color(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>& point_cloud_sift = *cloud_Color;
	int total = sifts.points.size() / 10;
	for (int i = 0; i < total; ++i) {
		pcl::PointWithScale pointSift = sifts.points[rand() % total];
		pcl::PointXYZRGB point;
		for (int j = 0; j < cloud->points.size(); ++j) {
			point = cloud->points[j];
			/*if (pointSift.x == point.x && pointSift.y == point.y
			 && pointSift.z == point.z) {*/
			//TODO comparacion un poco basica - por alguna razon los keypoints de sift no tienen la misma coordenada q los puntos de la nube original
			if (sqrt(
					pow(pointSift.x - point.x, 2)
							+ pow(pointSift.y - point.y, 2)
							+ pow(pointSift.z - point.z, 2)) < 0.01) {
				point_cloud_sift.push_back(point);
				/*std::cout << point_cloud_sift.back().x << " "
				 << point_cloud_sift.back().y << " "
				 << point_cloud_sift.back().z << std::endl;*/
				break;
			}
		}
	}

	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the SHOT descriptors for each point.
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(
			new pcl::PointCloud<pcl::SHOT352>());

	//if (cloud_Color->size() > 3) {
	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud_Color);
	normalEstimation.setRadiusSearch(0.05);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(
			new pcl::search::KdTree<pcl::PointXYZRGB>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);
	std::vector<int> indices2;
	pcl::removeNaNNormalsFromPointCloud(*normals, *normals, indices2);
	std::cout << "Point cloud normals size after removing NaNs: "
			<< normals->points.size() << std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFinal(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < indices2.size(); ++i) {
		cloudFinal->push_back(cloud_Color->points[indices2[i]]);
	}

	// SHOT estimation object.
	pcl::SHOTEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> shot;
	shot.setInputCloud(cloudFinal);
	shot.setInputNormals(normals);
	// The radius that defines which of the keypoint's neighbors are described.
	// If too large, there may be clutter, and if too small, not enough points may be found.
	shot.setRadiusSearch(0.04);

	shot.compute(*descriptors);
	//}
	if (descriptors->size() == 0) {
		pcl::SHOT352 des;
		for (int i = 0; i < 352; ++i) {
			des.descriptor[i] = 0;
			if (i < 9)
				des.rf[i] = 0;
		}
		descriptors->push_back(des);
	}
	return descriptors;
}

std::vector<int> matchSHOT(pcl::PointCloud<pcl::SHOT352>::Ptr descriptors1,
		pcl::PointCloud<pcl::SHOT352>::Ptr descriptors2) {

	// A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
	pcl::KdTreeFLANN<pcl::SHOT352> matching =
			new pcl::KdTreeFLANN<pcl::SHOT352>(false);
	pcl::KdTree<pcl::SHOT352>::PointCloudConstPtr ptr_descriptors1(
			descriptors1);
	matching.setInputCloud(descriptors1);
	// A Correspondence object stores the indices of the query and the match,
	// and the distance/weight.
	std::vector<int> correspondence(1);

	// Check every descriptor computed for the scene.
	for (size_t i = 0; i < descriptors2->points.size(); ++i) {
		std::vector<int> neighbors(1);
		std::vector<float> squaredDistances(1);
		if (pcl_isfinite(descriptors2->at(i).descriptor[0])) {
			// Find the nearest neighbor (in descriptor space)...
			int neighborCount = matching.nearestKSearch(descriptors2->at(i), 1,
					neighbors, squaredDistances);
			// ...and add a new correspondence if the distance is less than a threshold
			// (SHOT distances are between 0 and 1, other descriptors use different metrics).
			if (neighborCount == 1 && squaredDistances[0] < 0.25f) {
				//correspondence.push_back(neighbors[0], static_cast<int>(i), squaredDistances[0]);
				correspondence.push_back(neighbors[0]);
			}
		} else
			std::cout << "Found NaN in SHOT descriptors" << std::endl;
	}
	//std::cout << "Found " << correspondence.size() << " correspondences\n";
	return correspondence;
}

/*int numSameDescriptors(pcl::PointCloud<RIFT32>::Ptr descriptors) {
 int repe = 0;
 std::vector<RIFT32> repeatedEl;
 for (int i = 0; i < descriptors->points.size(); ++i) {
 RIFT32 des = descriptors->points[i];
 if (!repeatedEl.empty())
 if (std::find(repeatedEl.begin(), repeatedEl.end(), des)
 != repeatedEl.end()) {
 repe++;
 } else {
 repeatedEl.push_back(des);
 }
 }
 return repe;
 }*/

double computeVariance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	double variance = 0;
	//Computation of mean of y coordinate
	double mean = 0;

	return variance;
}

float distanceCentroids(std::vector<float> c1, std::vector<float> c2) {
	float distance = pow(c1[0] - c2[0], 2) + pow(c1[1] - c2[1], 2)
			+ pow(c1[2] - c2[2], 2);
	//std::cout << "centroid x: " << c1[0] << " and " << c2[0] << std::endl;
	//std::cout << "centroid y: " << c1[1] << " and " << c2[1] << std::endl;
	//std::cout << "centroid z: " << c1[1] << " and " << c2[2] << std::endl;
	return sqrt(distance);
}

std::vector<int> closestCentroids(std::vector<float> centroid1,
		std::vector<std::vector<float> > centroids2) {
	std::vector<int> correspondence(3);
	std::vector<float> distances(centroids2.size());
	for (int i = 0; i < centroids2.size(); ++i) {
		distances.push_back(distanceCentroids(centroid1, centroids2[i]));
	}
	std::sort(distances.begin(), distances.end());
	correspondence.push_back(distances[0]);
	correspondence.push_back(distances[1]);
	correspondence.push_back(distances[2]);
	return correspondence;
}

int closestCentroid(std::vector<float> centroid1,
		std::vector<std::vector<float> > centroids2, std::set<int> indices) {
	int indice = -1;
	std::vector<float> distances(centroids2.size());
	float minDis = 1000000000000;
	float actDis = 0;
	for (int i = 0; i < centroids2.size(); ++i) {
		//If centroid i is not previously found
		if (indices.find(i) == indices.end()) {
			actDis = distanceCentroids(centroid1, centroids2[i]);
			if (actDis < minDis) {
				minDis = actDis;
				indice = i;
			}
		}
	}
	return indice;
}

bool performICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud1,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud2) {
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setMaximumIterations(20);
	icp.setInputSource(point_cloud1);
	icp.setInputTarget(point_cloud2);
	pcl::PointCloud<pcl::PointXYZRGB> Final;
	icp.align(Final);
	//If ICP converge hasConverged() = 1 -- fitness score tells nothing about how good was the matching
	std::cout << "has converged:" << icp.hasConverged()
			<< " ICP fitness score: " << icp.getFitnessScore() << std::endl;
	//std::cout << icp.getFinalTransformation() << std::endl;

	if (icp.hasConverged() == 1)
		std::cout << "ICP has converged; starting comparison of point clouds"
				<< std::endl;
	else
		std::cout
				<< "ICP has not converged; point clouds too much different to perform a specific comparison"
				<< std::endl;
	return icp.hasConverged() == 1;
}

double computeSimilarity(char** argv, std::vector<int> pcl_filename_indices) {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud1_ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud1_nonoise_ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>& point_cloud1 = *point_cloud1_ptr;
	if (pcl::io::loadPLYFile(argv[pcl_filename_indices[0]], point_cloud1)
			== -1) {
		std::cerr << "Was not able to open file \""
				<< argv[pcl_filename_indices[0]] << "\".\n";
		return -2;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud2_ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud2_nonoise_ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>& point_cloud2 = *point_cloud2_ptr;
	if (pcl::io::loadPLYFile(argv[pcl_filename_indices[1]], point_cloud2)
			== -1) {
		std::cerr << "Was not able to open file \""
				<< argv[pcl_filename_indices[1]] << "\".\n";
		return -2;
	}

	std::ofstream myfile;
	myfile.open("../../PointCloudComparatorResults/results.txt");
	myfile << "Results of comparison between " << argv[pcl_filename_indices[0]]
			<< " and " << argv[pcl_filename_indices[1]]
			<< "\n--------------------------------------------------------------------------------\n\n";

	std::vector<int> indices2;
	pcl::removeNaNFromPointCloud(*point_cloud1_ptr, *point_cloud1_ptr,
			indices2);
	indices2.clear();
	pcl::removeNaNFromPointCloud(*point_cloud2_ptr, *point_cloud2_ptr,
			indices2);
	indices2.clear();

	//ICP
	if (icp)
		if (!performICP(point_cloud1_ptr, point_cloud2_ptr)) {
			myfile << "----------------------------" << "\n\n";
			myfile
					<< "ICP could not match the point clouds. They are probably too dissimilar.\n Brief comparison:\n";
			if (point_cloud1_ptr->size() > point_cloud2_ptr->size()) {
				std::cout << "PCL1 has more points: "
						<< point_cloud1_ptr->size() << " over: "
						<< point_cloud2_ptr->size() << std::endl;
				myfile << "PCL1 has more points: " << point_cloud1_ptr->size()
						<< " over: " << point_cloud2_ptr->size() << "\n";
			} else if (point_cloud2_ptr->size() > point_cloud1_ptr->size()) {
				std::cout << "PCL2 has more points: "
						<< point_cloud2_ptr->size() << " over: "
						<< point_cloud1_ptr->size() << std::endl;
				myfile << "PCL2 has more points: " << point_cloud2_ptr->size()
						<< " over: " << point_cloud1_ptr->size() << "\n";
			} else {
				std::cout << "Both PCL have the same number of points"
						<< std::endl;
				myfile << "Both PCL have the same number of points" << "\n";
			}
			myfile.close();
			return -1;
		} else {
			std::cout
					<< "ICP has converged. Point clouds segmentation is as follows"
					<< std::endl;
			myfile
					<< "ICP has converged. Point clouds segmentation is as follows: \n";
		}

	//Segmentation
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_pcl_1;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_pcl_2;
	if (euclidean) {
		clusters_pcl_1 = euclidean_cluster_segmentation(point_cloud1_ptr);
		clusters_pcl_2 = euclidean_cluster_segmentation(point_cloud2_ptr);
	} else {
		clusters_pcl_1 = region_growing_segmentation(point_cloud1_ptr, false);
		clusters_pcl_2 = region_growing_segmentation(point_cloud2_ptr, false);
	}

	myfile << "Number of points of PCL 1: " << point_cloud1_ptr->size() << "\n";
	myfile << "Number of points of PCL 2: " << point_cloud2_ptr->size() << "\n";
	myfile << "++++++++++++++++++++++++++++++++++++++++\n";
	myfile << "Number of clusters of PCL 1: " << clusters_pcl_1.size() << "\n";
	myfile << "Number of clusters of PCL 2: " << clusters_pcl_2.size() << "\n";

	//TODO precompute normal estimation to pass it to the other functions
	pcl::PointCloud<RIFT32>::Ptr des1Second;
	pcl::PointCloud<RIFT32>::Ptr des2Second;
	std::vector<int> correspondences;
	std::vector<int> correspondences2;
	//For each cluster in 1, there is a correspondence in 2
	std::vector<int> matches;
	int maxCor2 = 0;
	int clusMax;
	std::vector<int> numberDescriptors1;
	std::vector<pcl::PointCloud<RIFT32>::Ptr> descriptors2;
	std::vector<std::vector<float> > centroidsPCL2;

	myfile << "\n------------------------------------\n";
	myfile << "Information of clusters of PCL2:\n";
	myfile << "------------------------------------\n";

	for (int j = 0; j < clusters_pcl_2.size(); ++j) {
		myfile << "PCL2 cluster " << j << ":\n";
		myfile << "\tNumber of points: " << clusters_pcl_2[j]->size() << "\n";
		if (clusters_pcl_2[j]->size() > 30000)
			descriptors2.push_back(processRIFTwithSIFT(clusters_pcl_2[j]));
		else
			descriptors2.push_back(processRIFT(clusters_pcl_2[j]));
		myfile << "\tNumber of descriptors: " << descriptors2[j]->size()
				<< "\n";
		//Centroid computation
		std::vector<float> centroid;
		centroid.push_back(0);
		centroid.push_back(0);
		centroid.push_back(0);
		for (int l = 0; l < clusters_pcl_2.at(j)->size(); ++l) {
			pcl::PointXYZRGB punto = clusters_pcl_2.at(j)->at(l);
			centroid[0] += punto.x;
			centroid[1] += punto.y;
			centroid[2] += punto.z;
		}
		centroid[0] = centroid[0] / clusters_pcl_2.at(j)->size();
		centroid[1] = centroid[1] / clusters_pcl_2.at(j)->size();
		centroid[2] = centroid[2] / clusters_pcl_2.at(j)->size();
		centroidsPCL2.push_back(centroid);
		myfile << "\tCoordinates of centroid: [" << centroid[0] << ","
				<< centroid[1] << "," << centroid[2] << "]\n";
	}

	myfile << "\n------------------------------------\n";
	myfile << "Information of clusters of PCL 1:\n";
	myfile << "------------------------------------\n";

	std::vector<pcl::PointCloud<RIFT32>::Ptr> descriptors1;
	std::vector<std::vector<float> > centroidsPCL1;
	for (int i = 0; i < clusters_pcl_1.size(); ++i) {
		myfile << "PCL1 cluster " << i << ":\n";
		myfile << "\tNumber of points: " << clusters_pcl_1[i]->size() << "\n";
		//des1VHF = processVHF(clusters_pcl_1[i]);
		if (clusters_pcl_1[i]->size() > 30000)
			des1Second = processRIFTwithSIFT(clusters_pcl_1[i]);
		else
			des1Second = processRIFT(clusters_pcl_1[i]);
		myfile << "\tNumber of descriptors: " << des1Second->size() << "\n";
		descriptors1.push_back(des1Second);
		//descriptors1SHOT.push_back(processSHOT(clusters_pcl_1[i]));
		numberDescriptors1.push_back(des1Second->size());

		//Centroid computation
		std::vector<float> centroid;
		centroid.push_back(0);
		centroid.push_back(0);
		centroid.push_back(0);
		for (int l = 0; l < clusters_pcl_1.at(i)->size(); ++l) {
			pcl::PointXYZRGB punto = clusters_pcl_1.at(i)->at(l);
			centroid[0] += punto.x;
			centroid[1] += punto.y;
			centroid[2] += punto.z;
		}
		centroid[0] = centroid[0] / clusters_pcl_1.at(i)->size();
		centroid[1] = centroid[1] / clusters_pcl_1.at(i)->size();
		centroid[2] = centroid[2] / clusters_pcl_1.at(i)->size();
		centroidsPCL1.push_back(centroid);
		myfile << "\tCoordinates of centroid: [" << centroid[0] << ","
				<< centroid[1] << "," << centroid[2] << "]\n";
		maxCor2 = 0;
		matches.push_back(-1);
		//for (int j = 0; j < clusters_pcl_2.size(); ++j) {
		//if (distanceCentroids(centroid, centroidsPCL2[j]) < 0.05) {
		//Get 3 closests centroids
		std::set<int> indices;
		std::vector<int> closest;
		int closest1 = closestCentroid(centroid, centroidsPCL2, indices);
		indices.insert(closest1);
		closest.push_back(closest1);
		int closest2 = closestCentroid(centroid, centroidsPCL2, indices);
		indices.insert(closest2);
		closest.push_back(closest2);
		int closest3 = closestCentroid(centroid, centroidsPCL2, indices);
		closest.push_back(closest3);

		for (int j = 0; j < 3; ++j) {
			int closeCentroid = closest[j];
			if (closeCentroid != -1) {
				des2Second = descriptors2[closeCentroid];
				if (!des2Second->empty() and !des1Second->empty()
						and des2Second->points.size() > 3
						and des1Second->points.size() > 3) {
					double coef = (descriptors2[closeCentroid]->points.size()
							/ des1Second->points.size());
					if (coef > 0.5 && coef < 2) {
						std::cout << "Des pcl 1: " << des1Second->points.size()
								<< std::endl;
						std::cout << "Des pcl 2: " << des2Second->points.size()
								<< std::endl;
						correspondences = matchRIFTFeaturesKnn(des1Second,
								des2Second);
						/*correspondences2 = matchSHOT(descriptors1SHOT[i],
						 descriptors2SHOT[j]);*/

						if (des1Second->points.size()
								> des2Second->points.size()) {
							std::cout
									<< "Percentage of RIFT correspondences of clusters "
									<< i << " and " << closeCentroid << " is: "
									<< correspondences.size()
											/ des1Second->points.size() * 100
									<< std::endl;
							//If at least half of the descriptors match, we assume its a correspondence
							if (correspondences.size()
									/ des1Second->points.size() > 0.5
									and correspondences.size() > maxCor2) {
								maxCor2 = correspondences.size();
								clusMax = closeCentroid;
								matches[i] = clusMax;
								std::cout << "Match accepted" << std::endl;
							}
						} else {
							std::cout
									<< "Percentage of RIFT correspondences of clusters "
									<< i << " and " << closeCentroid << " is: "
									<< correspondences.size()
											/ des2Second->points.size() * 100
									<< std::endl;
							//If at least half of the descriptors match, we assume its a correspondence
							if (correspondences.size()
									/ des2Second->points.size() > 0.5
									and correspondences.size() > maxCor2
									/*and correspondences2.size()
									 / descriptors2SHOT[j]->size()
									 > 0.5*/) {
								maxCor2 = correspondences.size();
								clusMax = closeCentroid;
								matches[i] = clusMax;
								//std::cout << "Match accepted" << std::endl;
							}
						}
					}
				}
			} else
				std::cout << "No closer centroid found" << std::endl;
		}
	}

	myfile << "\n------------------------------------\n";
	myfile << "Information of matches of clusters of PCL 1 and PCL 2:\n";
	myfile << "------------------------------------\n";

	int pcl1 = 0;
	int pcl2 = 0;
	int numMatches = 0;
	for (int i = 0; i < matches.size(); ++i) {
		//For each match
		if (matches[i] != -1) {
			++numMatches;
			//Similarity of segments
			int simil1 = 0;
			int simil2 = 0;
			myfile << "\tMatched cluster " << i << " of PCL 1 with cluster "
					<< matches[i] << " of PCL 2:\n";
			if (seeClusters) {
				//Show both segments
				pcl::visualization::CloudViewer viewer2("Cluster viewer");
				viewer2.showCloud(clusters_pcl_1.at(i));
				while (!viewer2.wasStopped()) {
				}
				pcl::visualization::CloudViewer viewer3("Cluster viewer");
				viewer3.showCloud(clusters_pcl_2[matches[i]]);
				while (!viewer3.wasStopped()) {
				}
			}
			//Number of points comparison
			if (clusters_pcl_1.at(i)->points.size()
					> clusters_pcl_2.at(matches[i])->points.size()) {
				++simil1;
				myfile << "\t\tSegment of PCL 1 has more points: "
						<< clusters_pcl_1.at(i)->points.size() << " over: "
						<< clusters_pcl_2.at(matches[i])->points.size() << "\n";
			} else if (clusters_pcl_1.at(i)->points.size()
					< clusters_pcl_2.at(matches[i])->points.size()) {
				++simil2;
				myfile << "\t\tSegment of PCL 2 has more points: "
						<< clusters_pcl_2.at(matches[i])->points.size()
						<< " over: " << clusters_pcl_1.at(i)->points.size()
						<< "\n";
			} else {
				myfile << "\t\tBoth segments have the same number of points: "
						<< clusters_pcl_1.at(i)->points.size() << "\n";
			}

			//Number of descriptors comparison
			if (numberDescriptors1[i] > descriptors2[matches[i]]->size()) {
				++simil1;
				myfile << "\t\tSegment of PCL 1 has more descriptors: "
						<< numberDescriptors1[i] << " over: "
						<< descriptors2[matches[i]]->size() << "\n";
			} else if (numberDescriptors1[i]
					< descriptors2[matches[i]]->size()) {
				++simil2;
				myfile << "\t\tSegment of PCL 2 has more descriptors: "
						<< descriptors2[matches[i]]->size() << " over: "
						<< numberDescriptors1[i] << "\n";
			} else
				myfile
						<< "\t\tBoth segments have the same number of descriptors: "
						<< numberDescriptors1[i] << "\n";

			//TODO descriptors repeated??
			//int repetidos1 = numSameDescriptors(descriptors1RIFT[i]);
			//int repetidos2 = numSameDescriptors(descriptors2[i]);

			//TODO Surface analysis: variance
			//double var1 = computeVariance(clusters_pcl_1[i]);
			//double var2 = computeVariance(clusters_pcl_2[matches[i]]);

			//Color based segmentation of both match to find which has more elements with different colors.
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> colors_segments_pcl1 =
					color_growing_segmentation(clusters_pcl_1[i]);
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> colors_segments_pcl2 =
					color_growing_segmentation(clusters_pcl_2[matches[i]]);
			if (colors_segments_pcl1.size() > colors_segments_pcl2.size()) {
				myfile
						<< "\t\tSegment of PCL 1 has more elements based on color differences: "
						<< colors_segments_pcl1.size() << " over "
						<< colors_segments_pcl2.size() << "\n";
				/*std::cout
				 << "Segment of PCL 1 has more elements based on color differences: "
				 << colors_segments_pcl1.size() << " over "
				 << colors_segments_pcl2.size() << std::endl;*/
				++simil1;
			} else if (colors_segments_pcl1.size()
					< colors_segments_pcl2.size()) {
				myfile
						<< "\t\tSegment of PCL 2 has more elements based on color differences: "
						<< colors_segments_pcl1.size() << " over "
						<< colors_segments_pcl2.size() << "\n";
				/*std::cout
				 << "Segment of PCL 2 has more elements based on color differences: "
				 << colors_segments_pcl1.size() << " over "
				 << colors_segments_pcl2.size() << std::endl;*/
				++simil2;
			} else {
				myfile
						<< "\t\tSegment of PCL 1 and segment of PCL 2 have the same number of elements based on color differences: "
						<< colors_segments_pcl1.size() << "\n";
				/*std::cout
				 << "Segment of PCL 1 and segment of PCL 2 have the same number of elements based on color differences: "
				 << colors_segments_pcl1.size() << std::endl;*/
			}

			//Information of general pcl for each segment
			if (simil1 > simil2)
				++pcl1;
			else if (simil1 < simil2)
				++pcl2;
		} else {
			std::cout << "No match" << std::endl;
			myfile << "\t\tCluster " << i
					<< " of PCL 1 has no match in PCL 2\n";
		}
		myfile
				<< "      ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	\n";
	}
	//Information of general pcl
	//Number of segments
	if (clusters_pcl_1.size() > clusters_pcl_2.size())
		++pcl1;
	else if (clusters_pcl_2.size() > clusters_pcl_1.size())
		++pcl2;

	std::cout << std::endl;
	myfile << "Total number of matches found: " << numMatches << "\n\n";

	if (noise) {
		//Noise measurement: percentage of points removed (noisy points) with respect to original pcl
		//Noise pcl 1
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setInputCloud(point_cloud1_ptr);
		sor.setMeanK(50);
		sor.setStddevMulThresh(1.5);
		sor.filter(*point_cloud1_nonoise_ptr);
		/*pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
		 outrem.setInputCloud(point_cloud1_ptr);
		 outrem.setRadiusSearch(0.8);
		 outrem.setMinNeighborsInRadius(2);
		 outrem.filter(*point_cloud1_nonoise_ptr);*/
		double noise1 = (point_cloud1_ptr->size()
				- point_cloud1_nonoise_ptr->size()) / point_cloud1_ptr->size();
		//Noise pcl 2
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor2;
		sor2.setInputCloud(point_cloud2_ptr);
		sor2.setMeanK(50);
		sor2.setStddevMulThresh(1.5);
		sor2.filter(*point_cloud2_nonoise_ptr);
		/*pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem2;
		 outrem.setInputCloud(point_cloud2_ptr);
		 outrem.setRadiusSearch(0.8);
		 outrem.setMinNeighborsInRadius(2);
		 outrem.filter(*point_cloud2_nonoise_ptr);*/
		double noise2 = (point_cloud2_ptr->size()
				- point_cloud2_nonoise_ptr->size()) / point_cloud2_ptr->size();
		myfile
				<< "----------------------------------------\n Noise analysis: \n";
		if (noise1 > noise2) {
			std::cout << "PCL1 has more noisy points: (%) " << noise1 * 100
					<< " over: (%) " << noise2 * 100 << std::endl;
			myfile << "\tPCL1 has more noisy points: (%) " << noise1 * 100
					<< " over: (%) " << noise2 * 100 << "\n";
			++pcl2;
		} else if (noise1 < noise2) {
			std::cout << "PCL2 has more noisy points: (%) " << noise2 * 100
					<< " over: (%) " << noise1 * 100 << std::endl;
			myfile << "\tPCL2 has more noisy points: (%) " << noise2 * 100
					<< " over: (%) " << noise1 * 100 << "\n";
			++pcl1;
		} else {
			std::cout << "Both pcl have the same percentage of noisy points: "
					<< noise1 * 100 << std::endl;
			myfile << "Both pcl have the same percentage of noisy points: "
					<< noise1 * 100 << "\n";
		}
	}
	std::cout << std::endl;
	std::cout << "----------------------------" << std::endl;
	std::cout << "score pcl1: " << pcl1 << std::endl;
	std::cout << "score pcl2: " << pcl2 << std::endl;
	myfile << "\n----------------------------" << "\n\n";
	myfile << "Total score pcl1: " << pcl1 << "\n";
	myfile << "Total score pcl2: " << pcl2 << "\n";
	myfile.close();

	if (pcl1 > pcl2)
		return 1;
	else if (pcl1 < pcl2)
		return 2;
	else
		return 0;

}

// --------------
// -----Main-----
// --------------
int main(int argc, char** argv) {
// --------------------------------------
// -----Parse Command Line Arguments-----
// --------------------------------------
	std::cout << "------------------------------------" << std::endl;
	if (pcl::console::find_argument(argc, argv, "-h") >= 0) {
		printUsage();
		return 1;
	}
	if (pcl::console::find_argument(argc, argv, "-v") >= 0) {
		seeClusters = true;
		std::cout << "Visualization of clusters is on." << std::endl;
	} else
		std::cout << "Visualization of clusters is off." << std::endl;

	if (pcl::console::find_argument(argc, argv, "-n") >= 0) {
		noise = true;
		std::cout << "Noise analysis is on." << std::endl;
	} else
		std::cout << "Noise analysis is off." << std::endl;

	if (pcl::console::find_argument(argc, argv, "-i") >= 0) {
		icp = true;
		std::cout << "ICP matching pre-comparison is on." << std::endl;
	} else
		std::cout << "ICP matching pre-comparison is off." << std::endl;

	if (pcl::console::find_argument(argc, argv, "-e") >= 0) {
		euclidean = true;
		std::cout
				<< "Euclidean cluster segmentation was selected as main segmentation algorithm."
				<< std::endl;
	} else
		std::cout
				<< "Region growing segmentation was selected (default) as main segmentation algorithm."
				<< std::endl;

	std::cout << "------------------------------------" << std::endl;
	std::vector<int> pcl_filename_indices =
			pcl::console::parse_file_extension_argument(argc, argv, "ply");

	double similarity = computeSimilarity(argv, pcl_filename_indices);

	std::cout << "--------------------------------\n" << std::endl;
	if (similarity == 1) {
		cout << "The first point cloud has more information" << std::endl;
		/*std::cout
		 << "Points that are in the first pcl, that are not in the other:"
		 << std::endl;
		 spatial_change_detection(argv[pcl_filename_indices[1]],
		 argv[pcl_filename_indices[0]]);*/
	} else if (similarity == 2) {
		cout << "The second point cloud has more information" << std::endl;
		/*std::cout
		 << "Points that are in the second pcl, that are not in the other:"
		 << std::endl;
		 spatial_change_detection(argv[pcl_filename_indices[0]],
		 argv[pcl_filename_indices[1]]);*/
	} else if (similarity == 0)
		cout << "Both point clouds have the same information" << std::endl;

	std::cout << "--------------------------------\n" << std::endl;

	return 1;
}
