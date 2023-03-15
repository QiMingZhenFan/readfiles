#pragma once
#ifndef NDT_IMU_LOCALIZATION_INCLUDE_NDT_UTILITY_H_
#define NDT_IMU_LOCALIZATION_INCLUDE_NDT_UTILITY_H_

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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

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

#include <ndt_cpu/NormalDistributionsTransform.h>


using PointType = pcl::PointXYZ;

class NdtMatching{
    ros::NodeHandle nh;

    std::string pointCloudTopic;
    std::string imuOdomTopic;
    std::string ndtOdomTopic;
    std::string lidarFrame_;

    ros::Subscriber subImuOdom;
    ros::Subscriber subPointCloud;
    ros::Subscriber sub_initial_pose_;
    ros::Subscriber sub_fake_initial_pose_;
    ros::Publisher pubNdtOdometry;
    ros::Publisher pubGlobalMap;
    ros::Publisher pubCurPointCloud;

    std::string map_path_;
    pcl::PointCloud<PointType>::Ptr global_map_;
    pcl::PointCloud<PointType>::Ptr global_map_downsample_;

    pcl::PointCloud<PointType>::Ptr current_pointcloud_;
    // pcl::NormalDistributionsTransform<PointType, PointType> ndt_;
    cpu::NormalDistributionsTransform<PointType, PointType> ndt_;
    Eigen::Matrix4f initial_guess_;

    std::mutex initial_guess_mutux_;

    bool map_load_ = false;
    double odom_timestamp_ = 0.0;
    bool initial_pose_set_ = false;
    bool initial_pose_used_ = false;

    double ndt_epsilon_;
    double ndt_step_size_;
    int ndt_max_iteration_;
    double ndt_resolution_;
    double ndt_currentpoints_leafsize_;
    bool use_imu_;
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    NdtMatching();
    void InitialPoseHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odom_msg);
    void OdometryHandler(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void PointCloudHandler(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg);
    void LoadGlobalMap();
    void FakeInitialPoseHandler(const geometry_msgs::PoseStamped::ConstPtr& odom_msg);
    bool GetIMUUseFlag(){
        return use_imu_;
    }
};

#endif