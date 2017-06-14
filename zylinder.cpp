#include <boost/thread.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/features/normal_3d.h>

#define VIS
#ifdef VIS
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#endif

typedef pcl::PointXYZ  PointT;

ros::Publisher pub_plane, pub_plane2, pub_cyl, pub_coeff;

pcl::PassThrough<PointT> pass;
pcl::NormalEstimation<PointT, pcl::Normal> ne;
pcl::SACSegmentationFromNormals<PointT, pcl::Normal> segn;
//	pcl::SACSegmentation<PointT> segn;
pcl::ExtractIndices<pcl::Normal> extract_normals;
pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

boost::shared_ptr<pcl::PointCloud<PointT> > 		cloud_filtered (new pcl::PointCloud<PointT>);
boost::shared_ptr<pcl::PointCloud<pcl::Normal> > 	cloud_normals (new pcl::PointCloud<pcl::Normal>);
boost::shared_ptr<pcl::PointCloud<PointT> > 		cloud_without_plane (new pcl::PointCloud<PointT>);
boost::shared_ptr<pcl::PointCloud<pcl::Normal> > 	cloud_normals_wo_plane (new pcl::PointCloud<pcl::Normal>);
boost::shared_ptr<pcl::ModelCoefficients > 			coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
boost::shared_ptr<pcl::PointIndices > 				inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

boost::condition_variable cond;
boost::mutex mut;
bool data_ready;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input) {

	sensor_msgs::PointCloud2 transformed;

	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*input, *cloud);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);


	seg.setInputCloud(cloud);
	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		//    return (-1);
	}

	//ROS_INFO_STREAM("Planar model:");
	/*ROS_INFO_STREAM("Model coefficients: " << coefficients->values[0] << " "
			<< coefficients->values[1] << " "
			<< coefficients->values[2] << " "
			<< coefficients->values[3]);*/
	//ROS_INFO_STREAM("Model inliers: " << inliers->indices.size ());
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	
	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	
	extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
	extract.filter (*cloud_out);
	
	//cloud_out->header.stamp = ros::Time::now ();
	//cloud_out->header.frame_id = "/kinect1_rgb_optical_frame";
	
	//pub_plane.publish (cloud_out);
	
	//ROS_INFO_STREAM("next models:");
	
	ROS_INFO_STREAM("PointCloud has: " << cloud->points.size () << " data points.");

	//ROS_INFO_STREAM("Filtering...");
	// Build a passthrough filter to remove spurious NaNs
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, 1.5);
	pass.filter (*cloud_filtered);
	ROS_INFO_STREAM("PointCloud after filtering has: " << cloud_filtered->points.size () << " data points.");
	
	//ROS_INFO_STREAM("Estimating point normals...");
	// Estimate point normals
	ne.setSearchMethod (tree);
	ne.setInputCloud (cloud_filtered);
	ne.setKSearch (50);
	ne.compute (*cloud_normals);
	
	// Create the segmentation object for the planar model and set all the parameters
	segn.setOptimizeCoefficients (true);
	segn.setModelType (pcl::SACMODEL_NORMAL_PLANE);
//	segn.setModelType (pcl::SACMODEL_PLANE);
	segn.setNormalDistanceWeight (0.1); // 0.1
	segn.setMethodType (pcl::SAC_RANSAC);
	segn.setMaxIterations (100);
	segn.setDistanceThreshold (0.05); // 0.02
	segn.setInputCloud (cloud_filtered);
	segn.setInputNormals (cloud_normals);

	//ROS_INFO_STREAM("Obtaining plane inliers...");
	// Obtain the plane inliers and coefficients
	segn.segment (*inliers_plane, *coefficients_plane);
	//ROS_INFO_STREAM("Plane coefficients: " << *coefficients_plane);

	//ROS_INFO_STREAM("Extracting plane inliers...");
	// Extract the planar inliers from the input cloud
	extract.setInputCloud (cloud_filtered);
	extract.setIndices (inliers_plane);
	extract.setNegative (false);


	//ROS_INFO_STREAM("Removing plane inliers...");
	// Remove the planar inliers, extract the rest
	extract.setNegative (true);
	extract.filter (*cloud_without_plane);

//	pub_plane.publish (cloud_without_plane);

	//ROS_INFO_STREAM("Extracting normals...");
	extract_normals.setNegative (true);
	extract_normals.setInputCloud (cloud_normals);
	extract_normals.setIndices (inliers_plane);
	extract_normals.filter (*cloud_normals_wo_plane);

	// Create the segmentation object for cylinder segmentation and set all the parameters
	segn.setOptimizeCoefficients (true);
	segn.setModelType (pcl::SACMODEL_CYLINDER);
	segn.setMethodType (pcl::SAC_RRANSAC);
	segn.setNormalDistanceWeight (0.01);
//	segn.setMaxIterations (10000);
	segn.setMaxIterations (500);
	segn.setDistanceThreshold (0.02);
	segn.setRadiusLimits (0.02, 0.06);
	segn.setInputCloud (cloud_without_plane);
	segn.setInputNormals (cloud_normals_wo_plane);

	ROS_INFO_STREAM("Obtaining cylinder inliers...");
	// Obtain the cylinder inliers and coefficients
	segn.segment (*inliers_cylinder, *coefficients_cylinder);
	ROS_INFO_STREAM("Cylinder coefficients: " << *coefficients_cylinder);

//	// Write the cylinder inliers to disk
	extract.setInputCloud (cloud_without_plane);
	extract.setIndices (inliers_cylinder);
	extract.setNegative (false);
	ROS_INFO_STREAM("Extracting cylinder inliers...");
	pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
	extract.filter (*cloud_cylinder);
	pub_cyl.publish(*cloud_cylinder);
}

	int main (int argc, char** argv) {
	// Initialize ROS
	ros::init (argc, argv, "planar_segmentation");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud
	pub_plane = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("segmented_plane", 1);
	pub_plane2 = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("segmented_plane2", 1);
	pub_cyl = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("cylinder", 1);

	// Create a ROS publisher for the output model coefficients
	
	//pub_coeff = nh.advertise<pcl::ModelCoefficients> ("seg_plane_model_coeff", 1);
	ROS_INFO("Start to spin");
	ros::spin();
}
