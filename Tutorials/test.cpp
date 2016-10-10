#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>

#include <Eigen/Geometry>

typedef pcl::PointXYZ PointType;

void transformPC(pcl::PointCloud<PointType>::Ptr source_cloud, pcl::PointCloud<PointType>::Ptr transformed_cloud)
{
	//  METHOD #2: Using a Affine3f
	float theta = M_PI/4; // The angle of rotation in radians
	
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	// Define a translation of 2.5 meters on the x axis.
	transform_2.translation() << 2.5, 0.0, 0.0;

	// The same rotation matrix as before; theta radians arround Z axis
	transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));

	// Print the transformation
	printf ("\nMethod #2: using an Affine3f\n");
	std::cout << transform_2.matrix() << std::endl;

	// You can either apply transform_1 or transform_2; they are the same
	pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);
	
}


int main (int argc, char** argv)
{
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> sourceClouds;
	sourceClouds.resize(4);
	//~ vector < PointCloud<PointXYZ>::Ptr, Eigen::aligned_allocator <PointCloud <PointXYZ>::Ptr > > sourceClouds;
	pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr	cloud_a (new pcl::PointCloud<pcl::PointXYZ>), cloud_b (new pcl::PointCloud<pcl::PointXYZ>),  cloud_c (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr	cloud_d (new pcl::PointCloud<pcl::PointXYZ>);


	// Fill in the cloud data
	pcl::PCDReader reader;
	reader.read ("table_scene_lms400.pcd", *cloud_blob);
	std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud_blob);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_filtered_blob);

	// Convert to the templated PointCloud
	pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	
	// Optional
	seg.setOptimizeCoefficients (true);
	
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (0.01);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	int i = 0, nr_points = (int) cloud_filtered->points.size ();
	
	// While 30% of the original cloud is still there
	while (cloud_filtered->points.size () > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
		  std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
		  break;
		}

		// Extract the inliers
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter(*sourceClouds[i]);
		//~ sourceClouds.push_back(cloud_p);

		// Create the filtering object
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_filtered.swap (cloud_f);
		i++;
		
	}
	
    //~ cout << "Point Cloud " << 1 << "has got " << sourceClouds[0]->size() << " Points" << endl;
    //~ cout << "Point Cloud " << 2 << "has got " << sourceClouds[1]->size() << " Points" << endl;
	
	//Visual
	pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

	// Define R,G,B colors for the point cloud
	//~ viewer.addPointCloud (sourceClouds[0], "original");
	
	//~ pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (sourceClouds[0], 0, 255, 20); //White
	//~ viewer.addPointCloud (sourceClouds[0], source_cloud_color_handler, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (sourceClouds[1], 230, 20, 20); // Red
	viewer.addPointCloud (sourceClouds[1], transformed_cloud_color_handler, "transformed_cloud");

	viewer.addCoordinateSystem (1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
	//viewer.setPosition(800, 400); // Setting visualiser window position
	
	while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce ();
	}
	

	return (0);
}


