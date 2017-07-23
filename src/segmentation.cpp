/* \author Adrian Arroyo - adr.arroyo.perez@gmail.com */

#include "../include/segmentation.h"

int planar_segmentation(std::string filename) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>& point_cloud = *point_cloud_ptr;
	if (pcl::io::loadPLYFile(filename, point_cloud) == -1) {
		std::cerr << "Was not able to open file \"" << filename << "\".\n";
	}
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud(point_cloud_ptr);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0) {
		PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
		return (-1);
	}
	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
			<< coefficients->values[1] << " " << coefficients->values[2] << " "
			<< coefficients->values[3] << std::endl;

	std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	for (size_t i = 0; i < inliers->indices.size(); ++i) {
		std::cerr << inliers->indices[i] << "    "
				<< point_cloud_ptr->points[inliers->indices[i]].x << " "
				<< point_cloud_ptr->points[inliers->indices[i]].y << " "
				<< point_cloud_ptr->points[inliers->indices[i]].z << std::endl;
		cloud_cluster->points.push_back(
				point_cloud_ptr->points[inliers->indices[i]]);

	}
	cloud_cluster->width = cloud_cluster->points.size();
	cloud_cluster->height = 1;
	cloud_cluster->is_dense = true;
	pcl::visualization::CloudViewer viewer2("Original point_cloud - q to quit");
	viewer2.showCloud(point_cloud_ptr);
	while (!viewer2.wasStopped()) {
	}
	pcl::visualization::CloudViewer viewer(
			"Planar segmentation of pcl - q to quit");
	viewer.showCloud(cloud_cluster);
	while (!viewer.wasStopped()) {
	}
	return (0);
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> euclidean_cluster_segmentation(
		std::string filename) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>& point_cloud = *point_cloud_ptr;
	if (pcl::io::loadPLYFile(filename, point_cloud) == -1) {
		std::cerr << "Was not able to open file \"" << filename << "\".\n";
	}

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	vg.setInputCloud(point_cloud_ptr);
	vg.setLeafSize(0.025f, 0.025f, 0.025f);
	vg.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: "
			<< cloud_filtered->points.size() << " data points." << std::endl;

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(
			new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PLYWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02);

	int i = 0, nr_points = (int) cloud_filtered->points.size();
	while (cloud_filtered->points.size() > 0.3 * nr_points) {
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0) {
			std::cout
					<< "Could not estimate a planar model for the given dataset."
					<< std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);
		std::cout << "PointCloud representing the planar component: "
				<< cloud_plane->points.size() << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*cloud_filtered = *cloud_f;
	}

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
			new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance(0.05); // 2cm
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(250000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_pcl;
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it =
			cluster_indices.begin(); it != cluster_indices.end(); ++it) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(
				new pcl::PointCloud<pcl::PointXYZRGB>);
		for (std::vector<int>::const_iterator pit = it->indices.begin();
				pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(point_cloud_ptr->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: "
		 << cloud_cluster->points.size() << " data points." << std::endl;
		 pcl::visualization::CloudViewer viewer("Cloud cluster - q to quit");
		 viewer.showCloud(cloud_cluster);
		 while (!viewer.wasStopped()) {
		 }
		std::vector<int> indices2;
		pcl::removeNaNFromPointCloud(*cloud_cluster, *cloud_cluster, indices2);
		clusters_pcl.push_back(cloud_cluster);
		indices2.clear();
	}

	return clusters_pcl;
}

int color_growing_segmentation(std::string filename) {
	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<
			pcl::search::Search<pcl::PointXYZRGB> >(
			new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>& point_cloud = *point_cloud_ptr;
	if (pcl::io::loadPLYFile(filename, point_cloud) == -1) {
		std::cerr << "Was not able to open file \"" << filename << "\".\n";
	}

	pcl::IndicesPtr indices(new std::vector<int>);
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(point_cloud_ptr);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);

	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	reg.setInputCloud(point_cloud_ptr);
	reg.setIndices(indices);
	reg.setSearchMethod(tree);
	/*Defaults:
	 reg.setDistanceThreshold(10);
	 reg.setPointColorThreshold(6);
	 reg.setRegionColorThreshold(5);
	 reg.setMinClusterSize(600);*/
	reg.setDistanceThreshold(15);
	reg.setPointColorThreshold(3);
	reg.setRegionColorThreshold(1);
	reg.setMinClusterSize(400);

	std::vector<pcl::PointIndices> clusters;
	reg.extract(clusters);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud =
			reg.getColoredCloud();
	pcl::visualization::CloudViewer viewer("Cluster viewer");
	pcl::PLYWriter writer;
	std::stringstream ss;
	ss << "color_growing_cloud.ply";
	writer.write<pcl::PointXYZRGB>(ss.str(), *colored_cloud, false);
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped()) {
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}

	return (0);
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> region_growing_segmentation(
		std::string filename) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>& point_cloud = *point_cloud_ptr;
	if (pcl::io::loadPLYFile(filename, point_cloud) == -1) {
		std::cerr << "Was not able to open file \"" << filename << "\".\n";
	}
	std::vector<int> indices2;
	pcl::removeNaNFromPointCloud(*point_cloud_ptr, *point_cloud_ptr, indices2);

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	vg.setInputCloud(point_cloud_ptr);
	vg.setLeafSize(0.025f, 0.025f, 0.025f);
	vg.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: "
			<< cloud_filtered->points.size() << " data points." << std::endl;

	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<
			pcl::search::Search<pcl::PointXYZRGB> >(
			new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud_filtered);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);

	pcl::IndicesPtr indices(new std::vector<int>);
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(cloud_filtered);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);

	pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
	/*Defaults
	 reg.setMinClusterSize(50);
	 reg.setMaxClusterSize(1000000);
	 reg.setSearchMethod(tree);
	 reg.setNumberOfNeighbours(30);
	 reg.setInputCloud(cloud_filtered);
	 //reg.setIndices (indices);
	 reg.setInputNormals(normals);
	 reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	 reg.setCurvatureThreshold(1.0);*/
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(100);
	reg.setInputCloud(cloud_filtered);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1);

	std::vector<pcl::PointIndices> clusters;
	reg.extract(clusters);

	std::cout << "Number of clusters is equal to " << clusters.size()
			<< std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size()
			<< " points." << endl;
	/*std::cout << "These are the indices of the points of the initial"
	 << std::endl << "cloud that belong to the first cluster:"
	 << std::endl;
	 int counter = 0;
	 while (counter < clusters[0].indices.size()) {
	 std::cout << clusters[0].indices[counter] << ", ";
	 counter++;
	 if (counter % 10 == 0)
	 std::cout << std::endl;
	 }
	 std::cout << std::endl;*/

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud =
			reg.getColoredCloud();
	pcl::PLYWriter writer;
	std::stringstream ss;
	ss << "region_growing_cloud.ply";
	writer.write<pcl::PointXYZRGB>(ss.str(), *colored_cloud, false);
	pcl::visualization::CloudViewer viewer("Cluster viewer");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped()) {
	}

	//Store clusters into new pcls and all the clusters in an array of pcls
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_pcl;
	for (int i = 0; i < clusters.size(); ++i) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(
				new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud_cluster->width = clusters[i].indices.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		for (int j = 0; j < clusters[i].indices.size(); ++j) {
			//Take the corresponding point of the filtered cloud from the indices for the new pcl
			cloud_cluster->push_back(
					cloud_filtered->at(clusters[i].indices[j]));
		}
		indices2.clear();
		pcl::removeNaNFromPointCloud(*cloud_cluster, *cloud_cluster, indices2);
		clusters_pcl.push_back(cloud_cluster);
	}

	return clusters_pcl;
}

int ground_segmentation(std::string filename) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& point_cloud = *point_cloud_ptr;
	if (pcl::io::loadPLYFile(filename, point_cloud) == -1) {
		std::cerr << "Was not able to open file \"" << filename << "\".\n";
	}

	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1(
			new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(point_cloud_ptr);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointIndicesPtr ground(new pcl::PointIndices);

	// Create the filtering object
	pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
	pmf.setInputCloud(cloud_filtered1);
	pmf.setMaxWindowSize(20);
	pmf.setSlope(1.0f);
	pmf.setInitialDistance(0.5f);
	pmf.setMaxDistance(3.0f);
	pmf.extract(ground->indices);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud_filtered1);
	extract.setIndices(ground);
	extract.filter(*cloud_filtered);

	std::cerr << "Ground cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZ>("samp11-utm_ground.ply", *cloud_filtered,
			false);

	// Extract non-ground returns
	extract.setNegative(true);
	extract.filter(*cloud_filtered);

	std::cerr << "Object cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	writer.write<pcl::PointXYZ>("samp11-utm_object.ply", *cloud_filtered,
			false);

	/*pcl::visualization::CloudViewer viewer("Cluster viewer");
	 viewer.showCloud(cloud_filtered);
	 while (!viewer.wasStopped()) {
	 }*/

	return (0);
}

/*int main(int argc, char** argv) {
 std::vector<int> pcl_filename_indices =
 pcl::console::parse_file_extension_argument(argc, argv, "ply");
 //std::cout << argv[pcl_filename_indices[0]];
 //planar_segmentation(argv[pcl_filename_indices[0]]);
 //cluster_segmentation(argv[pcl_filename_indices[0]]);
 //color_growing_segmentation(argv[pcl_filename_indices[0]]);
 //ground_segmentation(argv[pcl_filename_indices[0]]);
 std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clusters_pcl_1 =
 region_growing_segmentation(argv[pcl_filename_indices[0]]);

 for (int i=0; i< clusters_pcl_1.size(); ++i){
 pcl::visualization::CloudViewer viewer("Cluster viewer");
 viewer.showCloud(clusters_pcl_1[i]);
 while (!viewer.wasStopped()) {
 }
 }

 /*Error con chair.ply:
 * Number of clusters is equal to 0
 First cluster has 0 points.
 segmentator: /usr/include/boost/smart_ptr/shared_ptr.hpp:646: typename boost::detail::sp_dereference<T>::type boost::shared_ptr<T>::operator*() const [with T = pcl::PointCloud<pcl::PointXYZRGB>; typename boost::detail::sp_dereference<T>::type = pcl::PointCloud<pcl::PointXYZRGB>&]: Assertion `px != 0' failed.
 Abortado (`core' generado)

 }*/
