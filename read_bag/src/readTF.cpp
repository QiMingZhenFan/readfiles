#include "geometry_msgs/Vector3.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/time.h"
#include "rosbag/message_instance.h"
#include "rosbag/query.h"
#include <algorithm>
#include <fstream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <string>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <tf/tfMessage.h>
#include <utility>
#include <vector>

#define foreach BOOST_FOREACH

using namespace std;

int main(int argc, char** argv){
    
    ros:ros::init(argc, argv, "readBag");
    ros::NodeHandle nh;
    rosbag::Bag bag;
    string file_name;
    // string folder_name = "";  // cannot use ----- in ~/.ros
    string folder_name = "/home/simulation/workspace/my_ws/src/data/";

    if (nh.getParam("file_name", file_name) == false){
        ROS_INFO_STREAM("please set the bag file name!");
        return 1;
    }

    try {
        bag.open(file_name, rosbag::BagMode::Read);
    } 
    catch (exception& e) {
        ROS_ERROR_STREAM("ERROR! Cannot open the bag!");
        return 1;
    }

    ofstream outFile_gt(folder_name + "trajectory_gt.csv", ios::out);
    if (!outFile_gt) {
        ROS_ERROR_STREAM("Cannot open the trajectory_gt.csv!");
        return 1;
    }

    ofstream outFile_aftmapped(folder_name + "trajectory_aftmapped.csv", ios::out);
    if (!outFile_aftmapped) {
        ROS_ERROR_STREAM("Cannot open the trajectory_aftmapped.csv!");
        return 1;
    }

    // rosbag::View view(bag);

    // ros::Time bag_begin_time = view.getBeginTime();
    // ros::Time bag_end_time = view.getEndTime();

    // ROS_INFO_STREAM("ROS bag time: " << (bag_end_time - bag_begin_time).toSec());

    vector<string> topics;
    topics.push_back(string("/tf"));  // must have '/'
    topics.push_back(string("/tf_static"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    string frame_id, child_frame_id;
    geometry_msgs::Vector3 translation;
    double time = 0;
    vector<pair<double, geometry_msgs::Vector3> > trajectory;
    vector<double> timestamp;
    int count = 0;
    foreach(rosbag::MessageInstance const m, view){
        tf::tfMessage::ConstPtr tfPtr = m.instantiate<tf::tfMessage>();
        if(tfPtr != NULL){
            // only one transformstamed in tf::message
            foreach(geometry_msgs::TransformStamped const geo, tfPtr->transforms) {
                // some frame_ids have '/' 
                child_frame_id = geo.child_frame_id;
                frame_id = geo.header.frame_id;
                time = geo.header.stamp.toSec();
                translation = geo.transform.translation; 
                // 'translation' can output directly
                // ROS_INFO_STREAM(frame_id << '\t' << child_frame_id << '\t' << translation); 

                if (frame_id == "/camera_init" and child_frame_id == "/for_evo_metric") {
                    timestamp.push_back(time);
                }
                if (frame_id == "map" and child_frame_id == "base_link_gt") {
                    trajectory.push_back(make_pair(time, translation));
                }
            }
        }
    }

    bag.close();
    ROS_INFO_STREAM("bag closed");

    for(auto data : trajectory){
        outFile_gt << data.first << '\t' << data.second.x << '\t' << data.second.y << '\t' << data.second.z << endl;
    }
    outFile_gt.close();
    ROS_INFO_STREAM("outFile_gt closed");

    for(auto data : timestamp){
        outFile_aftmapped << data << endl;
    }
    outFile_aftmapped.close();
    ROS_INFO_STREAM("outFile_aftmapped closed");

    ros::spin();
    return  0; 

}
