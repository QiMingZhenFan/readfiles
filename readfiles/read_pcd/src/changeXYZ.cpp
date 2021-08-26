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

using namespace std;

int main(int argc, char** argv)
{  
    ros::init(argc, argv, "readPcd");
    ros::NodeHandle nh;
    string foldername;
    string file_name[3] = {"surfaceMap.pcd","cornerMap.pcd","finalCloud.pcd"};

    if (nh.getParam("foldername", foldername) == false){
        ROS_INFO_STREAM("please set the target PCD file name!");
        return -1;
    }
    if (foldername.back() != '/') {
        foldername += '/';
    }
    ROS_INFO_STREAM("the pcd file folder is: " + foldername);
    ROS_INFO_STREAM("press any key to conntinue...");
    string key;
    cin >> key;

    int i = 2;
    while (i < 3) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new(new pcl::PointCloud<pcl::PointXYZ>);

        if(pcl::io::loadPCDFile<pcl::PointXYZ>(foldername + file_name[i], *cloud) == -1){
            ROS_ERROR_STREAM("cannot open the PCD file!");
            return -1;
        }

        for(auto &data : cloud->points)
        {
            // pcl::PointXYZ point;
            // point.x = data.z;
            // point.y = data.x;
            // point.z = data.y;
            // cloud_new->points.push_back(point);

            // 为什么这样不行？
            float temp;
            temp = data.z;
            data.z = data.y;
            data.y = data.x;
            data.x = temp;
        }
        // cloud_new->width = cloud_new->points.size();
        // cloud_new->height = 1;
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
