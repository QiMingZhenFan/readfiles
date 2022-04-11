#include "ros/init.h"
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Geometry>

using namespace std;
Eigen::Vector3d translation;
Eigen::Quaterniond rotation;

void transform_param_set(Eigen::Vector3d &translation, Eigen::Quaterniond &rotation);
void align_to_the_carla_coord(pcl::PointXYZ &point);

int main(int argc, char** argv)
{  
    ros::init(argc, argv, "readPcd");
    ros::NodeHandle nh;
    string foldername;
    string file_name[3] = {"surfaceMap.pcd","cornerMap.pcd","finalCloud.pcd"};
    int map_file_start_index = 0;

    if (nh.getParam("foldername", foldername) == false){
        ROS_INFO_STREAM("please set the target PCD file name!");
        return -1;
    }
    if (nh.getParam("map_file_start_index", map_file_start_index) == false){
        ROS_INFO_STREAM("please set the map_file_start_index!");
        return -1;
    }
    if (foldername.back() != '/') {
        foldername += '/';
    }
    ROS_INFO_STREAM("the pcd file folder is: " + foldername);
    ROS_INFO_STREAM("press any key to conntinue...");
    string key;
    cin >> key;

    transform_param_set(translation, rotation);
    int i = map_file_start_index;
    while (i < 3) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new(new pcl::PointCloud<pcl::PointXYZ>);

        if(pcl::io::loadPCDFile<pcl::PointXYZ>(foldername + file_name[i], *cloud) == -1){
            ROS_ERROR_STREAM("cannot open the PCD file!");
            return -1;
        }
        int count = 0;
        for(auto &data : cloud->points)
        {
            // count++;
            float temp;
            temp = data.z;
            data.z = data.y;
            data.y = data.x;
            data.x = temp;
            // align_to_the_carla_coord(data);
            // ROS_INFO_STREAM("processing file " << file_name[i] << ". finished points "<< count << " in total " << cloud->width);
        }
        pcl::io::savePCDFile(foldername + file_name[i], *cloud);

        ROS_INFO_STREAM(file_name[i] << " cloud height: " << cloud->height << "\twidth: " << cloud->width);

        ++i;

    }


    ROS_INFO_STREAM("ready!");

    // pcl::visualization::CloudViewer viewer("cloud viewer");
    // viewer.showCloud(cloud);

    // ros::spin();
    return 0;
}

/*
  08.26时期的机场坐标
  position: 
    x: -93.0018081665
    y: -22.9996166229
    z: -0.0236528962851
  orientation: 
    x: -0.0021667339934
    y: -0.000303305188324
    z: -0.991441788137
    w: 0.130531199353
*/
/*
  09.08后的bag起始坐标
  position: 
    x: -92.7017593384
    y: -20.9991703033
    z: -0.0236528590322
  orientation: 
    x: -0.00218778657177
    y: -1.78807609571e-05
    z: -0.999997606622
    w: 4.43330363281e-06

*/
void transform_param_set(Eigen::Vector3d &translation, Eigen::Quaterniond &rotation){
    translation = Eigen::Vector3d(-92.7017593384, -20.9991703033, -0.0236528590322);
    // in w,x,y,z
    rotation = Eigen::Quaterniond(4.43330363281e-06, -0.00218778657177, -1.78807609571e-05, -0.999997606622);
    // ROS_INFO_STREAM(rotation.matrix());
}
void align_to_the_carla_coord(pcl::PointXYZ &point){
    Eigen::Vector3d data(point.x, point.y,point.z);
    data = rotation * data + translation;
    point.x = data[0];
    point.y = data[1];
    point.z = data[2];
}