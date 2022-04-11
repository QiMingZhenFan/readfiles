
#include "ros/duration.h"
#include "ros/init.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/PointCloud2.h"
#include <iostream>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>  
#include <pcl_conversions/pcl_conversions.h>  

#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>

using namespace std;
typedef pcl::PointXYZ PointType;
string filename = "";
float x, y, z, theta;
int main(int argc, char** argv){

    ros::init(argc, argv, "icp_test");
    ros::NodeHandle nh;
    ros::Publisher pubbef = nh.advertise<sensor_msgs::PointCloud2>("/icp_bef", 100);
    ros::Publisher pubaft = nh.advertise<sensor_msgs::PointCloud2>("/icp_aft", 100);
    ros::Publisher pubori = nh.advertise<sensor_msgs::PointCloud2>("/icp_ori", 100);

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    if(nh.getParam("pcd_file_name", filename) == false){
        ROS_INFO_STREAM("please set the pcd file name!");
        return -1;
    }    
    if(nh.getParam("x", x) == false){
        ROS_INFO_STREAM("please set x!");
        return -1;
    }    
    if(nh.getParam("y", y) == false){
        ROS_INFO_STREAM("please set y!");
        return -1;
    }
    if(nh.getParam("z", z) == false){
        ROS_INFO_STREAM("please set z!");
        return -1;
    }
    if(nh.getParam("theta", theta) == false){
        ROS_INFO_STREAM("please set theta!");
        return -1;
    }
    theta = theta / 180.0 * M_PI; // must use radian
    if(pcl::io::loadPCDFile<PointType>(filename, *cloud) == -1){
        ROS_ERROR_STREAM("cannot open the PCD file!");
        return -1;
    }
    // transform this pointcloud
    Eigen::Affine3f transform_user_made = Eigen::Affine3f::Identity();
    transform_user_made.translation() << x, y, z;
    transform_user_made.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
    pcl::PointCloud<PointType>::Ptr transformed_cloud (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform_user_made);


    // ICP Settings
    static pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(50);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align clouds
    icp.setInputSource(cloud);
    icp.setInputTarget(transformed_cloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    ROS_INFO_STREAM("coveraged: " << icp.hasConverged() << "\tfitness score:" << icp.getFitnessScore());

    Eigen::Affine3f transform;
    transform = icp.getFinalTransformation();
    ROS_INFO_STREAM("transform user made: ");
    cout << transform_user_made.matrix() << endl;
    ROS_INFO_STREAM("transform calculated: ");
    cout << transform.matrix() << endl;

    sensor_msgs::PointCloud2 pcmsg;
    pcl::toROSMsg(*cloud, pcmsg);
    // above line can affect header info
    pcmsg.header.stamp = ros::Time::now();
    pcmsg.header.frame_id = "map";
    pubori.publish(pcmsg);

    pcl::toROSMsg(*transformed_cloud, pcmsg);
    pcmsg.header.stamp = ros::Time::now();
    pcmsg.header.frame_id = "map";
    pubbef.publish(pcmsg);

    pcl::PointCloud<PointType>::Ptr transformed_cloud_corrected (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud_corrected, transform.inverse());
    pcl::toROSMsg(*transformed_cloud_corrected, pcmsg);
    pcmsg.header.stamp = ros::Time::now();
    pcmsg.header.frame_id = "map";
    pubaft.publish(pcmsg);
    return 0;
}