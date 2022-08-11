#include "back_end.h"

#include "ros/ros.h"

// ros::Subscriber subOdom;
// ros::Subscriber subCloud;
// std::string odomTopic;

// void odomHandler(const nav_msgs::Odometry::ConstPtr &msg){

// }
// void laserCloudInfoHandler(){

// }

int main(int argc, char** argv){

    ros::init(argc, argv, "back_end");
    ros::NodeHandle nh;
    // nh.param<std::string>("odomTopic", odomTopic, "");
    // if(nh.getParam("odomTopic", odomTopic) == false){
    //     ROS_ERROR_STREAM("please set odomTopic param!");
    //     return -1;
    // }    

    // subCloud = nh.subscribe<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1, laserCloudInfoHandler);

    // subOdom =  nh.subscribe<nav_msgs::Odometry>(odomTopic, 20, odomHandler);
    ROS_INFO("\033[1;32m----> Back End Optimization Started.\033[0m");

    std::string bag_path = "/code/lidar-gnss-odoe.bag";
    std::string lidar_topic = "/carla/ego_vehicle/lidar";
    std::string pose_topic = "/Odometry";
    std::string gnss_topic = "/carla/ego_vehicle/gnss";

    back_end::BackEnd backend_g2o(bag_path, lidar_topic, pose_topic, gnss_topic);
    backend_g2o.Run();
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
}
