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

    ROS_INFO("\033[1;32m----> Back End Optimization Started.\033[0m");

    std::string bag_path;
    std::string lidar_topic;
    std::string pose_topic;
    std::string gnss_topic;

    nh.param<std::string>("bag_path", bag_path, "");
    nh.param<std::string>("lidar_topic", lidar_topic, "");
    nh.param<std::string>("pose_topic", pose_topic, "");
    nh.param<std::string>("gnss_topic", gnss_topic, "");

    back_end::BackEnd backend_g2o(bag_path, lidar_topic, pose_topic, gnss_topic);
    backend_g2o.Run();
    // ros::MultiThreadedSpinner spinner(4);
    // spinner.spin();
    ros::shutdown();
}
