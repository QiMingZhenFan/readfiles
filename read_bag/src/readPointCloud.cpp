#include "pcl/pcl_macros.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "sensor_msgs/PointCloud2.h"
#include "rosbag/message_instance.h"
#include "rosbag/query.h"
#include <algorithm>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <string>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <unistd.h>
#include <utility>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>            // this header file is different froem pcl/pcl_conversions.h
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#define foreach BOOST_FOREACH

using namespace std;

// 为了看点云一维的排列顺序
// velodyne: 一列一列排，每一列从下往上
// carla: 一行一行排列，每一行从左到右
void showPoints(rosbag::Bag &bag, vector<string> &topics){
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    string frame_id;
    double time;
    int height, width;
    int points_num;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::visualization::CloudViewer viewer("cloud viewer");
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer"));
    viewer->initCameraParameters();
    int v1(0), v2(0);//视口编号在这里设置两个视口
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("original", 10, 10, "v1 text", v1);
    viewer->addCoordinateSystem(1.0);

    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0, 0, 0, v2);
    viewer->addText("incremental", 10, 10, "v2 text", v2);
    viewer->addCoordinateSystem(1.0);


    foreach(rosbag::MessageInstance const m, view){
        sensor_msgs::PointCloud2ConstPtr pcPtr = m.instantiate<sensor_msgs::PointCloud2>();
        if(pcPtr != NULL){
            // some frame_ids have '/' 
            frame_id = pcPtr->header.frame_id;
            time = pcPtr->header.stamp.toSec();
            height = pcPtr->height;
            width = pcPtr->width;
            points_num = height * width;
            ROS_INFO_STREAM("height: " << height << "\twidth: " << width);
            // just convert the full point cloud
            pcl::fromROSMsg(*pcPtr, *pointCloud);
            
            // 拷贝构造函数是真拷贝，不是指向同一个地址
            pcl::PointCloud<pcl::PointXYZI> pointCloud_temp(*pointCloud);
            pcl::PointCloud<pcl::PointXYZI>::ConstPtr ptr(&pointCloud_temp);
            pointCloud_temp.points.clear();

            viewer->addPointCloud<pcl::PointXYZI>(pointCloud, "sample cloud1", v1);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud1");
            viewer->addPointCloud<pcl::PointXYZI>(ptr, "incremental cloud", v2);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "incremental cloud");


            for(auto m : pointCloud->points){
                pointCloud_temp.points.push_back(m);
                // viewer.showCloud(ptr);
                ROS_INFO_STREAM(ptr->points.size());
                viewer->removePointCloud("incremental cloud");
                viewer->addPointCloud<pcl::PointXYZI>(ptr, "incremental cloud", v2);
                viewer->spinOnce(100,true);  //刷新
            }
        }
    }
}

void publishTopicManual(ros::NodeHandle &nh, rosbag::Bag &bag, vector<string> &topics){
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_manual", 100);
    ROS_INFO("prepare to publish cloud manually...");
    ros::Rate rate(50);
    string temp;
    foreach(rosbag::MessageInstance const m, view){
        sensor_msgs::PointCloud2Ptr pcPtr = m.instantiate<sensor_msgs::PointCloud2>();
        pcPtr->header.frame_id = string("lidar");
        pubLaserCloud.publish(*pcPtr);
        ROS_INFO("publish once!");
        // rate.sleep();
        cin >> temp;
    }
}


int main(int argc, char** argv){
    
    ros:ros::init(argc, argv, "readPointCloud");
    ros::NodeHandle nh;
    rosbag::Bag bag;
    string file_name;

    if (nh.getParam("file_name", file_name) == false){
        ROS_INFO_STREAM("please set the bag file name!");
        return 1;
    }
    ROS_INFO_STREAM("the bag file is: " << file_name);
    try {
        bag.open(file_name, rosbag::BagMode::Read);
    } 
    catch (exception& e) {
        ROS_ERROR_STREAM("ERROR! Cannot open the bag!");
        return 1;
    }

    // rosbag::View view(bag);

    // ros::Time bag_begin_time = view.getBeginTime();
    // ros::Time bag_end_time = view.getEndTime();

    // ROS_INFO_STREAM("ROS bag time: " << (bag_end_time - bag_begin_time).toSec());

    vector<string> topics;
    // for velodyne
    topics.push_back(string("/velodyne_points"));  // must have '/'
    // for carla official ros bridge
    topics.push_back(string("/carla/lidar1/point_cloud"));  // must have '/'

    while (ros::ok()) {
        publishTopicManual(nh, bag, topics);
    }

    bag.close();
    ROS_INFO_STREAM("bag closed!");

    ros::spin();
    return  0; 

}



