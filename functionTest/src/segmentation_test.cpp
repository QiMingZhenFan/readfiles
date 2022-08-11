#include <thread>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

typedef pcl::PointXYZI PointT;
ros::Subscriber sub_point_cloud;
ros::Publisher pub_clustered_cloud, pub_cylinder_cloud;
std::string point_cloud_topic = "/carla/ego_vehicle/lidar"; 
bool cloud_seg_finished = false;
pcl::PointCloud<PointT>::Ptr cloud_aft_cluster_global;
std_msgs::Header header;

void PointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg){
    header = msg->header;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*msg, *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl;

    ros::WallTime bef(ros::WallTime::now());
    // Read in the cloud data
    pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<PointT> vg;
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    // vg.setInputCloud (cloud);
    // vg.setLeafSize (0.01f, 0.01f, 0.01f);
    // vg.filter (*cloud_filtered);
    cloud_filtered = cloud;
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl;

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud_filtered->size ();
    while (cloud_filtered->size () > 0.3 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloud_plane);
      std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloud_filtered = *cloud_f;
    }
    ros::WallTime aft_remove_planes(ros::WallTime::now());
    std::cout << "PointCloud aft remove the planar component: " << cloud_filtered->size () << " data points." << std::endl;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int num_of_kinds = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      // pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
      num_of_kinds++;
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
        // cloud_cluster->push_back ((*cloud_filtered)[*pit]);
        cloud_filtered->at(*pit).intensity = num_of_kinds;
      }
      // cloud_cluster->width = cloud_cluster->size ();
      // cloud_cluster->height = 1;
      // cloud_cluster->is_dense = true;

      // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    }
    cloud_aft_cluster_global = cloud_filtered;
    cloud_seg_finished = true;
    ros::WallTime aft_total(ros::WallTime::now());
    double plane_removal_time = (aft_remove_planes - bef).toSec();
    double total_time = (aft_total - bef).toSec();
    ROS_INFO_STREAM("planes removal time: " << plane_removal_time << "s.  total time: " << total_time << "s.");
}

void PublishCloud(){
    while(ros::ok()){
      if(cloud_seg_finished){
        cloud_seg_finished = false;
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud_aft_cluster_global, msg);
        msg.header = header;
        pub_clustered_cloud.publish(msg);

        ROS_INFO("clouds published!");
    }
  }
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "segmentation_test");
    ros::NodeHandle nh;

    sub_point_cloud = nh.subscribe<sensor_msgs::PointCloud2>(point_cloud_topic, 1, PointCloudCallBack, ros::TransportHints().tcpNoDelay());
    pub_clustered_cloud = nh.advertise<sensor_msgs::PointCloud2>("/clustered_cloud", 10);
    pub_cylinder_cloud = nh.advertise<sensor_msgs::PointCloud2>("/cylinder_clouds", 10);

    std::thread publish_clouds(PublishCloud);

    ros::spin();
    publish_clouds.join();
    return (0);
}