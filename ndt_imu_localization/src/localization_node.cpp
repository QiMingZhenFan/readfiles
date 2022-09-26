#include "imu_utility.h"
#include "ndt_utility.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "ndt_imu_localization");

    ROS_INFO("\033[1;32m----> NDT IMU Localization Started.\033[0m");

    NdtMatching ndt_matching;
    ndt_matching.LoadGlobalMap();
    ImuPreintegration imu_preintegration;
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
}