#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>

using PointType = pcl::PointXYZ;

class NdtMatching{
    ros::NodeHandle nh;

    std::string pointCloudTopic;
    std::string imuOdomTopic;

    ros::Subscriber subImuOdom;
    ros::Subscriber subPointCloud;
    ros::Publisher pubNdtOdometry;
    ros::Publisher pubGlobalMap;

    std::string map_path_;
    pcl::PointCloud<PointType>::Ptr global_map_;
    pcl::PointCloud<PointType>::Ptr global_map_downsample_;

    pcl::PointCloud<PointType>::Ptr current_pointcloud_;
    pcl::NormalDistributionsTransform<PointType, PointType> ndt_;
    Eigen::Matrix4f initial_guess_;

    bool map_load_ = false;
    double odom_timestamp_ = 0.0;

    NdtMatching():global_map_(new pcl::PointCloud<PointType>()){
        nh.param<std::string>("localization/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("localization/imuTopic", imuOdomTopic, "imu_correct");
        nh.param<std::string>("localization/mapPath", map_path_, "");

        subImuOdom = nh.subscribe<nav_msgs::Odometry>(imuOdomTopic, 5, &NdtMatching::OdometryHandler, this, ros::TransportHints().tcpNoDelay());
        subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &NdtMatching::PointCloudHandler, this, ros::TransportHints().tcpNoDelay());
        pubNdtOdometry = nh.advertise<nav_msgs::Odometry>("ndt/current_pose", 2000);
        pubGlobalMap = nh.advertise<sensor_msgs::PointCloud2>("global_map", 2000);

        global_map_downsample_.reset(new pcl::PointCloud<PointType>());
        current_pointcloud_.reset(new pcl::PointCloud<PointType>());
        initial_guess_ = Eigen::Matrix4f::Identity();

        // Setting minimum transformation difference for termination condition.
        ndt_.setTransformationEpsilon(0.01);
        ndt_.setStepSize(0.1);
        ndt_.setResolution(1.0);
        ndt_.setMaximumIterations(35);
    }

    void OdometryHandler(const nav_msgs::Odometry::ConstPtr& odom_msg){
        odom_timestamp_ = odom_msg->header.stamp.toSec();
        auto& t = odom_msg->pose.pose.position;
        auto& quaternion = odom_msg->pose.pose.orientation;
        Eigen::Quaternionf rotation(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        Eigen::Translation3f translation(t.x, t.y, t.z);

        // update initial guess
        initial_guess_.block<3, 3>(0, 0) = rotation.toRotationMatrix();
        initial_guess_.block<3, 1>(0, 3) = translation.vector();
    }

    void PointCloudHandler(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg){
        if(!map_load_)
            return;
        pcl::fromROSMsg(*pointcloud_msg, *current_pointcloud_);

        // downsample
        // ApproximateVoxelGrid -- faster but less precious
        pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter;
        approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
        approximate_voxel_filter.setInputCloud(current_pointcloud_);
        approximate_voxel_filter.filter(*current_pointcloud_);

        ndt_.align(*current_pointcloud_, initial_guess_);
        ROS_DEBUG_STREAM("Normal Distributions Transform has converged:" << ndt_.hasConverged()
            << " score: " << ndt_.getFitnessScore());

        static Eigen::Matrix4f output = Eigen::Matrix4f::Identity();
        output = ndt_.getFinalTransformation();

        // publish ndt odometry
        nav_msgs::Odometry ndt_odom;
        static unsigned int count = 0;
        ndt_odom.header.stamp = ros::Time::now();
        ndt_odom.header.frame_id = "map";
        ndt_odom.header.seq = count++;
        ndt_odom.pose.pose.position.x = output(0,3);
        ndt_odom.pose.pose.position.y = output(1,3);
        ndt_odom.pose.pose.position.z = output(2,3);
        Eigen::Quaternionf quat(output.block<3, 3>(0, 0));
        ndt_odom.pose.pose.orientation.w = quat.w();
        ndt_odom.pose.pose.orientation.x = quat.x();
        ndt_odom.pose.pose.orientation.y = quat.y();
        ndt_odom.pose.pose.orientation.z = quat.z();
        pubNdtOdometry.publish(ndt_odom);
    }

    void LoadGlobalMap(){
        if(map_path_.empty()){
            ROS_ERROR("Map path not set! cannot load global map in ndt module!");
            ros::shutdown();
        }
        if(pcl::io::loadPCDFile<PointType> (map_path_, *global_map_) == -1){
            ROS_ERROR("Cannot load global map in ndt module! map path: %s", map_path_.c_str());
            ros::shutdown();
        }
        ROS_INFO("Global Map has %ld points.", global_map_->size());

        // downsample
        pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter;
        approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
        approximate_voxel_filter.setInputCloud(global_map_);
        approximate_voxel_filter.filter(*global_map_downsample_);
        ROS_INFO("After downsample, global Map has %ld points.", global_map_->size());

        map_load_ = true;
        ndt_.setInputTarget(global_map_downsample_);

        sensor_msgs::PointCloud2 pc_to_pub;
        pcl::toROSMsg(*global_map_downsample_, pc_to_pub);
        pc_to_pub.header.frame_id = "map";

        ros::Rate rate(0.2);
        while(ros::ok()){
            if(pubNdtOdometry.getNumSubscribers() != 0){
                pc_to_pub.header.stamp = ros::Time::now();
                pubGlobalMap.publish(pc_to_pub);
            }
            rate.sleep();
        }
    }

};