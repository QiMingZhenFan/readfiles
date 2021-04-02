#include "ros/init.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

using namespace std;

bool readytogo = false;

void subHandler(const std_msgs::BoolConstPtr &data)
{
    readytogo = true;
    ROS_INFO_STREAM("ready to perform dataTOTUM... ");

}

int main(int argc, char** argv){

    ros::init(argc, argv, "dataTOTUM");
    ros::NodeHandle nh;

    ros::Subscriber subReadTFFinish = nh.subscribe<std_msgs::Bool>("/read_pcd_finished", 10, subHandler);

    while (!readytogo){
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    string folder_name;
    string filename[] = {"trajectory_aftmapped.csv", "trajectory_gt.csv"};

    if (nh.getParam("csv_folder_name", folder_name) == false){
        ROS_INFO_STREAM("please set the csv folder name!");
        return 1;
    }

    for(auto name : filename){
        ifstream inFile(folder_name + name, ios::in);
        if(!inFile){
            cout << "Cannot read file : " << name << endl;
            return 1;
        }
        vector<vector<string> > content;
        string line;
        while (getline(inFile, line)) {
            istringstream ss(line);
            vector<string> data;
            string temp;
            while (getline(ss, temp, '\t')) {
                data.push_back(temp);   // time x y z
            }
            content.push_back(data);
        }
        ofstream outFile(folder_name + name, ios::out);
        if(!outFile){
            cout << "Cannot write file : " << name << endl;
            return 1;
        }
        for (auto ptr = content.begin(); ptr != content.end(); ptr++) {
            outFile << ptr->data()[0] << ' ' << ptr->data()[1] << ' ' << ptr->data()[2] << ' ' << ptr->data()[3] << ' ' << 0 << ' ' << 0 << ' ' << 0 << ' ' << 0 << endl;
        }
        outFile.close();

    }
    ROS_INFO_STREAM("[3/3] convert to TUM finished! ");
    return 0;
}