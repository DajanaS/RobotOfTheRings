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

#include <move_base_msgs/MoveBaseAction.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#define VIS
#ifdef VIS
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#endif

typedef pcl::PointXYZRGB  PointT;

//tf::TransformListener listener;
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

class Zylinder
{
private:
	ros::NodeHandle n;
	tf::TransformListener listener;
	
public:
	Zylinder(ros::NodeHandle &nh):n(nh) {}
	~Zylinder(){}
	void get_pos(pcl::PointXYZRGB* pxls, int numb_pxl, move_base_msgs::MoveBaseGoal& goal)
	{
		move_base_msgs::MoveBaseGoal frame_goal;
		frame_goal.target_pose.pose.position.x = 0; 
		frame_goal.target_pose.pose.position.y = 0;
		for(int i = 0; i < numb_pxl; ++i)
		{
			frame_goal.target_pose.pose.position.x += pxls[i].x;
			frame_goal.target_pose.pose.position.y += pxls[i].y;
			frame_goal.target_pose.pose.position.z += pxls[i].z;
		}
		frame_goal.target_pose.pose.position.x /= numb_pxl;
		frame_goal.target_pose.pose.position.y /= numb_pxl;
		frame_goal.target_pose.pose.position.z /= numb_pxl;
		frame_goal.target_pose.header.frame_id = "camera_depth_optical_frame";
		frame_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
		ros::Time current_transform = ros::Time::now();
		ROS_INFO("Waiting for transform");
		listener.waitForTransform(frame_goal.target_pose.header.frame_id, "map",current_transform, ros::Duration(3.0));
		listener.getLatestCommonTime(frame_goal.target_pose.header.frame_id, "map", current_transform, NULL);
		frame_goal.target_pose.header.stamp = current_transform;
		ROS_INFO("Doing the transform");
		listener.transformPose("map", frame_goal.target_pose, goal.target_pose);
		//goal.target_pose.pose.position.y *= -1;
	}
		

	void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input) {
		//ROS_INFO("Got cloud");
		sensor_msgs::PointCloud2 transformed;

		// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		pcl::PCLPointCloud2 pcl_pc2;
		pcl_conversions::toPCL(*input,pcl_pc2);
		pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
		
		
		
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

		
			pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		  // Optional
		  seg.setOptimizeCoefficients (true);
		  // Mandatory
		  seg.setModelType (pcl::SACMODEL_PLANE);
		  seg.setMethodType (pcl::SAC_RANSAC);
		  seg.setMaxIterations (1000);
		  seg.setDistanceThreshold (0.01);
		  seg.setInputCloud (cloud_filtered);


		seg.setInputCloud(cloud);
		seg.segment (*inliers, *coefficients);

		if (inliers->indices.size () == 0)
		{
			PCL_ERROR ("Could not estimate a planar model for the given dataset.");
			//    return (-1);
		}
		
		pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>);
		
		// Create the filtering object
		pcl::ExtractIndices<PointT> extract;
		
		extract.setInputCloud (cloud);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_out);
		
		
		//cloud_out->header.stamp = ros::Time::now ();
		//cloud_out->header.frame_id = "/kinect1_rgb_optical_frame";
		
		//pub_plane.publish (cloud_out);
		
		//ROS_INFO_STREAM("next models:");
		
		//ROS_INFO_STREAM("PointCloud has: " << cloud->points.size () << " data points.");

		//ROS_INFO_STREAM("Filtering...");
		// Build a passthrough filter to remove spurious NaNs
		pass.setInputCloud (cloud);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0, 1.5);
		pass.filter (*cloud_filtered);
		//ROS_INFO_STREAM("PointCloud after filtering has: " << cloud_filtered->points.size () << " data points.");
		
		
		
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
		segn.setMaxIterations (1000);
		segn.setDistanceThreshold (0.05);
		segn.setRadiusLimits (0.01, 0.6);
		segn.setInputCloud (cloud_without_plane);
		segn.setInputNormals (cloud_normals_wo_plane);

		//ROS_INFO_STREAM("Obtaining cylinder inliers...");
		// Obtain the cylinder inliers and coefficients
		segn.segment (*inliers_cylinder, *coefficients_cylinder);
		//ROS_INFO_STREAM("Cylinder coefficients: " << *coefficients_cylinder);

	//	// Write the cylinder inliers to disk
		extract.setInputCloud (cloud_without_plane);
		extract.setIndices (inliers_cylinder);
		extract.setNegative (false);
		//ROS_INFO_STREAM("Extracting cylinder inliers...");
		pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
		extract.filter (*cloud_cylinder);
		
		int numb_blue = 0;
		int numb_red = 0;
		int numb_green = 0;
		int numb_yellow = 0;
		pcl::PointXYZRGB* blue = new pcl::PointXYZRGB[100];
		pcl::PointXYZRGB* red = new pcl::PointXYZRGB[100];
		pcl::PointXYZRGB* green = new pcl::PointXYZRGB[100];
		pcl::PointXYZRGB* yellow = new pcl::PointXYZRGB[100];
		
		pcl::PointCloud<pcl::PointXYZRGB>::iterator b1;
		for (b1 = cloud_cylinder->points.begin(); b1 < cloud_cylinder->points.end(); b1++)
		{
			uint32_t rgb = *reinterpret_cast<int*>(&b1->rgb);
			uint8_t r = (rgb >> 16) & 0x0000ff;
			uint8_t g = (rgb >> 8)  & 0x0000ff;
			uint8_t b = (rgb)       & 0x0000ff;
			if(b > 110 && r < 105 && g < 105)
			{
				 blue[numb_blue] = *b1;
				 numb_blue++;
			 }
			if(g > 110 && r < 105 && b < 105)
			{
				 green[numb_green] = *b1;
				 numb_green++;
			 }
			 if(r > 110 && b < 105 && g < 105)
			 {
				 red[numb_red] = *b1;
				 numb_red++;
			 }
			 if(r > 150 && g > 150 && b < 105)
			 {
				 yellow[numb_yellow] = *b1;
				 numb_yellow++;
			 }
			 bool publish = false;
			  move_base_msgs::MoveBaseGoal goal;
			 if(numb_blue > 7 && numb_green < 4 && numb_red < 4 && numb_yellow < 4)
			 {
				 cout << "Detected blue zylinder: numb:" << numb_blue << endl;
				 get_pos(blue, numb_blue, goal);
				 goal.target_pose.header.frame_id = "blue";
				 publish = true;
			 }
			 if(numb_green > 7 && numb_blue < 4 && numb_red < 4 && numb_yellow < 4)
			 {
				 cout << "Detected green zylinder: numb:" << numb_green << endl;
				 get_pos(green, numb_green, goal);
				 goal.target_pose.header.frame_id = "green";
				 publish = true;
			 }
			 if(numb_red > 7 && numb_green < 4 && numb_blue < 4 && numb_yellow < 4)
			 {
				 cout << "Detected red zylinder: numb:" << numb_red << endl;
				 get_pos(red, numb_red, goal);
				 goal.target_pose.header.frame_id = "red";
				 publish = true;
			 }
			 if(numb_yellow > 7 && numb_green < 4 && numb_red < 4 && numb_blue < 4)
			 {
				 cout << "Detected yellow zylinder: numb:" << numb_yellow << endl;
				 get_pos(red, numb_red, goal);
				 goal.target_pose.header.frame_id = "yellow";
				 publish = true;
			 }
			 if(publish) pub_cyl.publish(goal);
		}
	}
};


	int main (int argc, char** argv) {
	// Initialize ROS
	ros::init (argc, argv, "planar_segmentation");
	ros::NodeHandle nh;

	Zylinder sys(nh);
	cout <<"Created zylinder" << endl;
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/exercise6/voxelgrid", 1, &Zylinder::cloud_cb,&sys);
	//ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
	
	// Create a ROS publisher for the output point cloud
	pub_plane = nh.advertise<pcl::PointCloud<PointT> > ("segmented_plane", 1);
	pub_plane2 = nh.advertise<pcl::PointCloud<PointT> > ("segmented_plane2", 1);
	pub_cyl = nh.advertise<move_base_msgs::MoveBaseGoal> ("cylinder", 1);
	
	// Create a ROS publisher for the output model coefficients
	
	//pub_coeff = nh.advertise<pcl::ModelCoefficients> ("seg_plane_model_coeff", 1);
	ROS_INFO("Start to spin");
	ros::spin();
}
