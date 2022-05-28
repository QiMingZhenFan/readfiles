#include "geometry_msgs/Vector3.h"
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <tf/tfMessage.h>
#include <std_msgs/Bool.h>

#include <ros/ros.h>
#include "ros/init.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/time.h"
#include "rosbag/message_instance.h"
#include "rosbag/query.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "eigen_conversions/eigen_msg.h"

#include <algorithm>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

void calculateDelta(const Eigen::Isometry3d& gt, const Eigen::Isometry3d& slam, Eigen::Isometry3d& delta){
    delta = gt * slam.inverse();
}
int changePCDtoGlobalFrame(ros::NodeHandle &nh, string workingfolder, Eigen::Isometry3d &tansform);

int main(int argc, char** argv){
    
    ros::init(argc, argv, "readBag");
    ros::NodeHandle nh;
    ros::Publisher pubReadTFfinish = nh.advertise<std_msgs::Bool>("/read_tf_finished", 1);
    rosbag::Bag bag;
    string file_name;
    // string folder_name = "";  // cannot use ----- in ~/.ros
    string folder_name;
    string workingfolder;
    bool ifchangePCD = false;

    // get params
    if (nh.getParam("workingfolder", workingfolder) == false){
        ROS_INFO_STREAM("please set the workingfolder!");
        return 1;
    }
    if(workingfolder.back() != '/') workingfolder += '/';

    if (nh.getParam("bag_file_name", file_name) == false){
        ROS_INFO_STREAM("please set the bag file name!");
        return 1;
    }
    file_name = workingfolder + file_name;

    if (nh.getParam("csv_folder_name", folder_name) == false){
        ROS_INFO_STREAM("please set the csv folder name!");
        return 1;
    }
    if(folder_name.back() != '/') folder_name += '/';
    folder_name = workingfolder + folder_name;
    nh.param<bool>("ifchangePCD", ifchangePCD, false);



    try {
        bag.open(file_name, rosbag::BagMode::Read);
    } 
    catch (exception& e) {
        ROS_ERROR_STREAM("ERROR! Cannot open the bag!");
        return 1;
    }

    ofstream outFile_gt(folder_name + "trajectory_gt.csv", ios::out);
    if (!outFile_gt.is_open()) {
        ROS_ERROR_STREAM("Cannot open the trajectory_gt.csv!");
        return 1;
    }

    ofstream outFile_aftmapped(folder_name + "trajectory_aftmapped.csv", ios::out);
    if (!outFile_aftmapped.is_open()) {
        ROS_ERROR_STREAM("Cannot open the trajectory_aftmapped.csv!");
        return 1;
    }

    // rosbag::View view(bag);

    // ros::Time bag_begin_time = view.getBeginTime();
    // ros::Time bag_end_time = view.getEndTime();

    // ROS_INFO_STREAM("ROS bag time: " << (bag_end_time - bag_begin_time).toSec());

    // 1. read tf messages in bag files
    vector<string> topics;
    topics.push_back(string("/tf"));  // must have '/'
    topics.push_back(string("/tf_static"));
    topics.push_back(string("/lio_sam/mapping/path"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    string frame_id, child_frame_id;
    geometry_msgs::Transform transform;
    double time = 0;
    vector<pair<double, geometry_msgs::Transform> > trajectory_gt;
    vector<pair<double, geometry_msgs::Transform> > trajectory_slam;
    nav_msgs::Path global_path;  // for global path in lio-sam 
    for(rosbag::MessageInstance const m : view){
        tf::tfMessage::ConstPtr tfPtr = m.instantiate<tf::tfMessage>();
        if(tfPtr != NULL){
            // only one transformstamed in tf::message
            for(geometry_msgs::TransformStamped const geo : tfPtr->transforms) {
                // some frame_ids have '/' 
                child_frame_id = geo.child_frame_id;
                frame_id = geo.header.frame_id;
                time = geo.header.stamp.toSec();
                transform = geo.transform; 
                // ROS_INFO_STREAM(geo.header.stamp.sec << '\t' << geo.header.stamp.nsec << '\t' << setprecision(8) << geo.header.stamp.toSec());
                // 'translation' can output directly
                // ROS_INFO_STREAM(frame_id << '\t' << child_frame_id << '\t' << translation); 

                // if (frame_id == "odom" and child_frame_id == "base_link") {
                    // ROS_INFO_STREAM_ONCE("slam time: " << time);
                    // trajectory_slam.push_back(make_pair(time, transform));
                // }
                if (frame_id == "map" and child_frame_id == "base_link_gt") {
                    trajectory_gt.push_back(make_pair(time, transform));
                }
            }
        }
        nav_msgs::PathConstPtr navPtr = m.instantiate<nav_msgs::Path>();
        if(navPtr != nullptr){
            global_path = *navPtr;  // only keep the last message
        }
    }
    bag.close();
    ROS_INFO_STREAM("bag closed");
    geometry_msgs::Transform temp;
    for(auto pose : global_path.poses){
        temp.translation.x = pose.pose.position.x;
        temp.translation.y = pose.pose.position.y;
        temp.translation.z = pose.pose.position.z;
        temp.rotation = pose.pose.orientation;
        trajectory_slam.push_back(make_pair(pose.header.stamp.toSec(), temp));
    }

    // 2. get delta transform between gt and slam frames
    double first_slam_time = trajectory_slam[0].first;
    geometry_msgs::Transform first_transform_slam = trajectory_slam[0].second;
    geometry_msgs::Transform corresponding_transform_gt;

    const double epsilon = 5e-3;
    for(auto& data : trajectory_gt){
        if(fabs(data.first - first_slam_time) < epsilon){
            corresponding_transform_gt = data.second;
            break;
        }
        if(data.first > first_slam_time){
            // ROS_ERROR_STREAM("Cannot find corresponding pose in gt frame! first gt time: " << data.first << "   first slam time: " << first_slam_time);
            trajectory_slam.erase(trajectory_slam.begin());
            if(trajectory_slam.empty())
                return -1;
            else{
                first_slam_time = trajectory_slam[0].first;
                first_transform_slam = trajectory_slam[0].second;
            }
        }
    }
    Eigen::Isometry3d gt, slam, delta;
    tf::transformMsgToEigen(corresponding_transform_gt, gt);
    tf::transformMsgToEigen(first_transform_slam, slam);
    calculateDelta(gt, slam, delta);

    // 3. save data in TUM format
    for(auto& data : trajectory_gt){
        outFile_gt << setprecision(8) << data.first << ' ' << data.second.translation.x << ' ' << data.second.translation.y << ' ' << data.second.translation.z <<
                                    ' ' << data.second.rotation.x << ' ' << data.second.rotation.y << ' ' << data.second.rotation.z << ' ' << data.second.rotation.w << endl;
    }
    outFile_gt.close();
    ROS_INFO_STREAM("outFile_gt closed");

    Eigen::Isometry3d output;
    for(auto& data : trajectory_slam){
        // convert coordinates in slam frame to gt frame
        tf::transformMsgToEigen(data.second, output);
        output = delta * output;
        tf::transformEigenToMsg(output, data.second);
        outFile_aftmapped << setprecision(8) << data.first << ' ' << data.second.translation.x << ' ' << data.second.translation.y << ' ' << data.second.translation.z <<
                            ' ' << data.second.rotation.x << ' ' << data.second.rotation.y << ' ' << data.second.rotation.z << ' ' << data.second.rotation.w << endl;
    }
    outFile_aftmapped.close();
    ROS_INFO_STREAM("outFile_aftmapped closed");

    if(ifchangePCD){
        if(changePCDtoGlobalFrame(nh, workingfolder, delta) != 0){
            ROS_INFO("something wrong while change points in pcd file!");
            return -1;
        }
    }

    std_msgs::Bool ready;
    ready.data = true;
    pubReadTFfinish.publish(ready);
    ROS_INFO_STREAM("[1/3] read tf finished! ");
    return  0; 

}

// because pcd file comes from slam, it has local frame. Thus need to change points in pcd file into global frame
int changePCDtoGlobalFrame(ros::NodeHandle &nh, string workingfolder, Eigen::Isometry3d &tansform){
    string totalname, pcdname;
    if (nh.getParam("pcd_file_name", pcdname) == false){
        ROS_INFO_STREAM("please set the target PCD file name!");
        return -1;
    }
    totalname = workingfolder + pcdname;

    pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(totalname, *map) == -1){
        ROS_ERROR_STREAM("cannot open the PCD file!");
        return -1;
    }

    ROS_INFO_STREAM("cloud height: " << map->height << "\twidth: " << map->width);

    pcl::transformPointCloud(*map, *map, tansform.matrix());
    pcl::PointCloud<pcl::PointXYZ>::Ptr aft_filter(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> sor;//滤波处理对象
    double voxel_leaf_size = 0.2;
    sor.setInputCloud(map);
    sor.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);//设置滤波器处理时采用的体素大小的参数
    sor.filter(*aft_filter);

    pcl::io::savePCDFile(workingfolder + "changed_" + pcdname, *aft_filter);

    ROS_INFO_STREAM("the frame of pcd file has been changed!");
    return 0;
}