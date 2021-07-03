#include "ros/duration.h"
#include "ros/init.h"
#include "std_msgs/Bool.h"
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

bool readytogo = false;

void subHandler(const std_msgs::BoolConstPtr &data)
{
    readytogo = true;
    ROS_INFO_STREAM("ready to perform readPCD... ");

}

int main(int argc, char** argv)
{  
    ros::init(argc, argv, "readPcd");
    ros::NodeHandle nh;
    ros::Subscriber subReadTFFinish = nh.subscribe<std_msgs::Bool>("/read_tf_finished", 10, subHandler);
    ros::Publisher pubReadPcdfinish = nh.advertise<std_msgs::Bool>("/read_pcd_finished", 1);

    while (!readytogo){
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    string filename;
    string folder_name;

    if (nh.getParam("pcd_file_name", filename) == false){
        ROS_INFO_STREAM("please set the target PCD file name!");
        return -1;
    }

    if (nh.getParam("csv_folder_name", folder_name) == false){
        ROS_INFO_STREAM("please set the csv folder name!");
        return 1;
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1){
        ROS_ERROR_STREAM("cannot open the PCD file!");
        return -1;
    }

    ifstream inFile(folder_name + "trajectory_aftmapped.csv", ios::in);
    if(!inFile){
        ROS_ERROR_STREAM("Cannot open trajectory_aftmapped.csv ！");
        return -1;
    }
    string line;
    vector<vector<string> > content;
    while (getline(inFile, line)) {
        vector<string> lineCont;
        istringstream ss(line);
        string temp;
        while(getline(ss, temp, '\t')){
            lineCont.push_back(temp);
        }
        content.push_back(lineCont);
    }

    ROS_INFO_STREAM("cloud height: " << cloud->height << "\twidth: " << cloud->width);
    if(cloud->width != content.size()){
        ROS_ERROR_STREAM("size not match!");
        return 1;
    }
    int count = 0;
    for(auto ptr = cloud->begin(); ptr != cloud->end(); ++ptr, ++count){
        content[count].push_back(to_string(ptr->x));
        content[count].push_back(to_string(ptr->y));
        content[count].push_back(to_string(ptr->z));
    }
    ofstream outFile(folder_name + "trajectory_aftmapped.csv", ios::out);
    if(!outFile){
        ROS_ERROR_STREAM("Cannot write this file !");
        return 1;
    }
    for(auto data : content){
        outFile << data[0] << '\t' << data[1] << '\t' << data[2] << '\t' << data[3] << endl;
    }
    outFile.close();
    ROS_INFO_STREAM("data write finished! ");

    // pcl::visualization::CloudViewer viewer("cloud viewer");
    // viewer.showCloud(cloud);

    std_msgs::Bool ready;
    ready.data = true;
    pubReadPcdfinish.publish(ready);
    ROS_INFO_STREAM("[2/3] read pcd finished! ");
    return 0;
}
