#include <thread>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

typedef pcl::PointXYZ PointT;
ros::Subscriber sub_point_cloud;
ros::Publisher pub_plane_cloud, pub_cylinder_cloud;
std::string point_cloud_topic = "/carla/ego_vehicle/lidar"; 
bool cloud_seg_finished = false;
pcl::PointCloud<PointT>::Ptr cloud_cylinder_global;
pcl::PointCloud<PointT>::Ptr cloud_plane_global;
std_msgs::Header header;;

void PointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg){
    header = msg->header;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*msg, *cloud);
    ros::WallTime bef(ros::WallTime::now());
    // initialize variables
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
    ros::WallTime aft_initialize(ros::WallTime::now());

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 15);
    pass.filter (*cloud_filtered);
    std::cerr << "PointCloud after filtering has: " << cloud_filtered->size () << " data points." << std::endl;

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Write the planar inliers to disk
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_plane);
    std::cerr << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
    cloud_plane_global = cloud_plane;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.1);
    seg.setInputCloud (cloud_filtered2);
    seg.setInputNormals (cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_cylinder);
    if (cloud_cylinder->points.empty ()) 
      std::cerr << "Can't find the cylindrical component." << std::endl;
    else
    {
      std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->size () << " data points." << std::endl;
    }
    cloud_cylinder_global = cloud_cylinder;

    ros::WallTime aft_total(ros::WallTime::now());
    double initial_time = (aft_initialize - bef).toSec();
    double total_time = (aft_total - bef).toSec();
    ROS_INFO_STREAM("variable initialization time: " << initial_time << "s.  total time: " << total_time << "s.");
    cloud_seg_finished = true;
}

void PublishCloud(){
    while(ros::ok()){
      if(cloud_seg_finished){
        cloud_seg_finished = false;
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud_plane_global, msg);
        msg.header = header;
        pub_plane_cloud.publish(msg);

        pcl::toROSMsg(*cloud_cylinder_global, msg);
        msg.header = header;
        pub_cylinder_cloud.publish(msg);
        ROS_INFO("clouds published!");
    }
  }
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "plane_cylinder_extract");
    ros::NodeHandle nh;

    sub_point_cloud = nh.subscribe<sensor_msgs::PointCloud2>(point_cloud_topic, 1, PointCloudCallBack, ros::TransportHints().tcpNoDelay());
    pub_plane_cloud = nh.advertise<sensor_msgs::PointCloud2>("/plane_clouds", 10);
    pub_cylinder_cloud = nh.advertise<sensor_msgs::PointCloud2>("/cylinder_clouds", 10);

    std::thread publish_clouds(PublishCloud);

    ros::spin();
    publish_clouds.join();
    return (0);
}