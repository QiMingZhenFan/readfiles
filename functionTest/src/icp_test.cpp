
#include "ros/duration.h"
#include "ros/init.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/PointCloud2.h"
#include <iostream>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>  
#include <pcl/registration/gicp.h>  
#include <pcl/registration/ndt.h>  
#include <pcl_conversions/pcl_conversions.h>  

#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>

#include <teaser/ply_io.h>
#include <teaser/registration.h>

using namespace std;
typedef pcl::PointXYZ PointType;
string filename1 = "";
string filename2 = "";
float x, y, z, theta;

#define NOISE_BOUND 0.05

int main(int argc, char** argv){

    ros::init(argc, argv, "icp_test");
    ros::NodeHandle nh;
    ros::Publisher pubsrc = nh.advertise<sensor_msgs::PointCloud2>("/icp_src", 100);
    ros::Publisher pubunused = nh.advertise<sensor_msgs::PointCloud2>("/icp_unused", 100);
    ros::Publisher pubtar = nh.advertise<sensor_msgs::PointCloud2>("/icp_tar", 100);
    ros::Publisher pubtransformed = nh.advertise<sensor_msgs::PointCloud2>("/icp_transformed", 100);

    pcl::PointCloud<PointType>::Ptr cloud1(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud2(new pcl::PointCloud<PointType>);
    if(nh.getParam("pcd_file_name1", filename1) == false){
        ROS_INFO_STREAM("please set the pcd file name1!");
        return -1;
    }    
    if(nh.getParam("pcd_file_name2", filename2) == false){
        ROS_INFO_STREAM("please set the pcd file name2!");
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
    if(pcl::io::loadPCDFile<PointType>(filename1, *cloud1) == -1){
        ROS_ERROR_STREAM("cannot open the PCD file!");
        return -1;
    }
    if(pcl::io::loadPCDFile<PointType>(filename2, *cloud2) == -1){
        ROS_ERROR_STREAM("cannot open the PCD file!");
        return -1;
    }
    // transform this pointcloud
    Eigen::Affine3f transform_user_made = Eigen::Affine3f::Identity();
    // transform_user_made.translation() << x, y, z;
    // transform_user_made.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
    pcl::PointCloud<PointType>::Ptr transformed_cloud (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*cloud2, *transformed_cloud, transform_user_made);

    // icp & ndt
    {
        ros::Time t1 = ros::Time::now();
        // pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        // ndt.setTransformationEpsilon(0.01);//为终止条件设置最大转换差异
        // ndt.setStepSize(0.1);  //为More-Thuente线搜索设置最大步长
        // ndt.setResolution(0.2); //设置NDT网格结构的分辨率（VoxelGridCovariance）
        // ndt.setMaximumIterations(35); //设置匹配迭代的最大次数
        // ndt.setInputSource(cloud1);
        // ndt.setInputTarget(transformed_cloud);
        // ndt.align(*unused_result);
        // ROS_INFO_STREAM("coveraged: " << ndt.hasConverged() << "  fitness5 score:" << ndt.getFitnessScore(5) << "  fitness2 score:" << ndt.getFitnessScore(2));
        // Eigen::Affine3f transform_temp;
        // transform_temp = ndt.getFinalTransformation();

        // ICP Settings
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        // static pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(5);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(10);

        // Align clouds
        icp.setInputSource(cloud1);
        icp.setInputTarget(transformed_cloud);
        icp.align(*unused_result);

        ROS_INFO_STREAM("coveraged: " << icp.hasConverged() << "  fitness5 score:" << icp.getFitnessScore(5) << "  fitness2 score:" << icp.getFitnessScore(2));
        Eigen::Affine3f transform_temp;
        // transform_temp = icp.getFinalTransformation();
        transform_temp = Eigen::Affine3f::Identity();

        icp.setMaxCorrespondenceDistance(1);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(10);

        // Align clouds
        icp.setInputSource(cloud1);
        icp.setInputTarget(transformed_cloud);
        icp.align(*unused_result, transform_temp.matrix());
        ros::Time t2 = ros::Time::now();

        ROS_INFO_STREAM("coveraged: " << icp.hasConverged() << "  fitness5 score:" << icp.getFitnessScore(5) << "  fitness2 score:" << icp.getFitnessScore(2));
        ROS_INFO_STREAM("time consumed: " << (t2-t1).toSec());
        // Eigen::Affine3f transform;
        // transform = icp.getFinalTransformation();
        // ROS_INFO_STREAM("transform user made: ");
        // cout << transform_user_made.matrix() << endl;
        // ROS_INFO_STREAM("transform calculated: ");
        // cout << transform.matrix() << endl;
    }

    // teaser++
    {
        // int N1 = cloud1->size();

        // // Convert the point cloud to Eigen
        // Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N1);
        // for (size_t i = 0; i < N1; ++i) {
        //     src.col(i) << cloud1->at(i).x, cloud1->at(i).y, cloud1->at(i).z;
        // }

        // int N2 = cloud2->size();

        // // Convert the point cloud to Eigen
        // Eigen::Matrix<double, 3, Eigen::Dynamic> tgt(3, N1);
        // for (size_t i = 0; i < N1; ++i) {
        //     tgt.col(i) << cloud1->at(i).x, cloud1->at(i).y, cloud1->at(i).z;
        // }
        // ROS_INFO("am i here?");
        // // Run TEASER++ registration
        // // Prepare solver parameters
        // teaser::RobustRegistrationSolver::Params params;
        // // params.noise_bound = NOISE_BOUND;
        // params.cbar2 = 1;
        // params.estimate_scaling = false;
        // params.rotation_max_iterations = 100;
        // params.rotation_gnc_factor = 1.4;
        // params.rotation_estimation_algorithm =
        //     teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
        // params.rotation_cost_threshold = 0.005;

        // // Solve with TEASER++
        // teaser::RobustRegistrationSolver solver(params);
        // ROS_INFO("111111");

        // solver.solve(src, tgt);
        // ROS_INFO("2222222");

        // auto solution = solver.getSolution();
        // ROS_INFO("33333");

        // Eigen::Matrix4f transform_calc = Eigen::Matrix4f::Identity();
        // transform_calc.block<3,3>(0,0) = solution.rotation.cast<float>();
        // transform_calc.block<3,1>(0,3) = solution.translation.cast<float>();
        // pcl::transformPointCloud(*cloud1, *unused_result, transform_calc);
    }
    
    ros::Rate rate(1);
    while(ros::ok()){
        sensor_msgs::PointCloud2 pcmsg;
        pcl::toROSMsg(*cloud1, pcmsg);
        // above line can affect header info
        pcmsg.header.stamp = ros::Time::now();
        pcmsg.header.frame_id = "map";
        pubsrc.publish(pcmsg);
        // ROS_INFO_STREAM("pubsrc: " << cloud1->size());

        pcl::toROSMsg(*transformed_cloud, pcmsg);
        pcmsg.header.stamp = ros::Time::now();
        pcmsg.header.frame_id = "map";
        pubtar.publish(pcmsg);
        // ROS_INFO_STREAM("pubtar: " << transformed_cloud->size());

        pcl::toROSMsg(*unused_result, pcmsg);
        pcmsg.header.stamp = ros::Time::now();
        pcmsg.header.frame_id = "map";
        pubunused.publish(pcmsg);
        // ROS_INFO_STREAM("pubunused: " << unused_result->size());

        // pcl::PointCloud<PointType>::Ptr transformed_cloud_corrected (new pcl::PointCloud<PointType> ());
        // pcl::transformPointCloud(*transformed_cloud, *transformed_cloud_corrected, transform.inverse());
        // pcl::toROSMsg(*transformed_cloud_corrected, pcmsg);
        // pcmsg.header.stamp = ros::Time::now();
        // pcmsg.header.frame_id = "map";
        // pubtransformed.publish(pcmsg);
        rate.sleep();
    }
    
    return 0;
}