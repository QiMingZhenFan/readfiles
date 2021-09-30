#include "ros/init.h"
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Geometry>

using namespace std;
Eigen::Vector3d translation;
Eigen::Quaterniond rotation;

int main(int argc, char** argv)
{  
    ros::init(argc, argv, "downsample_pcd");
    ros::NodeHandle nh;
    string foldername;
    string file_name[3] = {"surfaceMap.pcd","cornerMap.pcd","finalCloud.pcd"};
    double voxel_leaf_size = 0.0;

    if (nh.getParam("foldername", foldername) == false){
        ROS_INFO_STREAM("please set the target PCD file name!");
        return -1;
    }
    if (nh.getParam("voxel_size", voxel_leaf_size) == false){
        ROS_INFO_STREAM("please set the voxel size for downsample!");
        return -1;
    }
    if (foldername.back() != '/') {
        foldername += '/';
    }
    ROS_INFO_STREAM("the pcd file folder is: " + foldername << ". leaf size is: " << voxel_leaf_size);
    ROS_INFO_STREAM("press any key to conntinue...");
    string key;
    cin >> key;

    int i = 2;
    while (i < 3) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr aft_filter(new pcl::PointCloud<pcl::PointXYZ>);

        if(pcl::io::loadPCDFile<pcl::PointXYZ>(foldername + file_name[i], *cloud) == -1){
            ROS_ERROR_STREAM("cannot open the PCD file!");
            return -1;
        }
        // 创建滤波器对象
        pcl::VoxelGrid<pcl::PointXYZ> sor;//滤波处理对象
        sor.setInputCloud(cloud);
        sor.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);//设置滤波器处理时采用的体素大小的参数
        sor.filter(*aft_filter);
        pcl::io::savePCDFile(foldername + "filtered_" + file_name[i], *aft_filter);

        ROS_INFO_STREAM(file_name[i] << " original cloud height: " << cloud->height << "\twidth: " << cloud->width);
        ROS_INFO_STREAM(file_name[i] << " filtered cloud height: " << aft_filter->height << "\twidth: " << aft_filter->width);

        ++i;

    }


    ROS_INFO_STREAM("ready!");

    // pcl::visualization::CloudViewer viewer("cloud viewer");
    // viewer.showCloud(cloud);

    // ros::spin();
    return 0;
}
