#include "ros/duration.h"
#include "ros/init.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include <iostream>
#include <fstream>

#include <sstream>
#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>

using namespace std;
ros::Subscriber subOdom;
ros::Subscriber subCloud;
string odomTopic;

inline double distance(geometry_msgs::Point& pos1, geometry_msgs::Point& pos2){
    double powdis = pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2) + pow(pos1.z - pos2.z, 2); 
    return pow(powdis, 0.5);
}

void odomHandler(nav_msgs::Odometry::ConstPtr &msg){

}
void laserCloudInfoHandler(){

}
int main(int argc, char** argv){

    ros::init(argc, argv, "back_end");
    ros::NodeHandle nh;
    // nh.param<std::string>("odomTopic", odomTopic, "");
    if(nh.getParam("odomTopic", odomTopic) == false){
        ROS_ERROR_STREAM("please set odomTopic param!");
        return -1;
    }    

    subCloud = nh.subscribe<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1, laserCloudInfoHandler);

    subOdom =  nh.subscribe<nav_msgs::Odometry>(odomTopic, 20, odomHandler);
    ROS_INFO("\033[1;32m----> Back End Optimization Started.\033[0m");

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
}