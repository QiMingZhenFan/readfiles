
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

// 为了实时计算里程计精度
// TODO：写成csv文件


using namespace std;
string version, gt_topic, traj_topic, folder_name;
float x, y, z, theta;

std::mutex mtx;
std::mutex vizmtx;

ros::Subscriber subGTPose;
std::deque<geometry_msgs::PoseStamped> gtQueue;
std::deque<geometry_msgs::PoseStamped> vizgtQueue;

ros::Subscriber subEstimatePose;
std::deque<nav_msgs::Odometry> estimateQueue;

ros::Publisher pubGTPoseInOdomFrame, pubGTPoseInOdomFrame_nav;


void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg){
    std::lock_guard<std::mutex> lock(mtx);
    estimateQueue.push_back(*odomMsg);
}


void gtHandler(const geometry_msgs::PoseStamped::ConstPtr& odomMsg){
    std::lock_guard<std::mutex> lock(mtx);
    gtQueue.push_back(*odomMsg);
    vizgtQueue.push_back(*odomMsg);

}

inline void mysleep(int time){
    std::chrono::milliseconds dura(time); // ms
    std::this_thread::sleep_for(dura);
}

bool first_gt_pose_set = false;
bool first_odom_pose_set = false;
bool find_corresponding_frame = false;
geometry_msgs::PoseStamped first_gt_pose, first_odom_pose;

void calcError(){
    std::unique_lock<std::mutex> lock(mtx);

    while (!estimateQueue.empty())
    {
        nav_msgs::Odometry cur_odom = estimateQueue.front();
        estimateQueue.pop_front();

        double cur_odom_time = cur_odom.header.stamp.toSec();
        geometry_msgs::PoseStamped cur_gt_pose;
        while(!gtQueue.empty()){
            double cur_gt_time = gtQueue.front().header.stamp.toSec();
            if(fabs(cur_gt_time - cur_odom_time) < 1e-5){  // found it
                if(!first_gt_pose_set){
                    first_gt_pose = gtQueue.front();
                    first_gt_pose_set = true;
                    // ROS_INFO_STREAM("first_gt_pose_set！");
                }
                if(!first_odom_pose_set){
                    first_odom_pose.pose = cur_odom.pose.pose;
                    first_odom_pose_set = true;
                    // ROS_INFO_STREAM("first_gt_pose_set！");
                }
                cur_gt_pose = gtQueue.front();
                gtQueue.pop_front();
                find_corresponding_frame = true;
                // ROS_INFO_STREAM("find corresponding gt pose!");
                break;
            }
            else if(cur_gt_time < cur_odom_time){
                gtQueue.pop_front();
                // ROS_WARN_STREAM("gt time < odom time!  " << setprecision(10) << cur_gt_time << ' ' << cur_odom_time);
                continue;
            }
            else{
                // ROS_WARN_STREAM("gt time > odom time!  " << setprecision(6) << cur_gt_time << ' ' << cur_odom_time);
                find_corresponding_frame = false;
                return;
            }
        }
        // lock.unlock(); // 退出临界区，其他线程可以继续读取queue


        if(!first_gt_pose_set || !first_gt_pose_set || !find_corresponding_frame){
            continue;
        }

        // 只使用tf API进行矩阵变换
        tf::Transform gt_first_tf, cur_gt_tf, cur_gtrel_tf;
        tf::poseMsgToTF(first_gt_pose.pose, gt_first_tf);
        tf::poseMsgToTF(cur_gt_pose.pose, cur_gt_tf);
        cur_gtrel_tf = gt_first_tf.inverse() * cur_gt_tf;

        tf::Transform odom_first_tf, cur_odom_tf, cur_odomrel_tf;
        tf::poseMsgToTF(cur_odom.pose.pose, cur_odom_tf);
        tf::poseMsgToTF(first_odom_pose.pose, odom_first_tf);
        cur_odomrel_tf = odom_first_tf.inverse() * cur_odom_tf;

        // calculate error in gt frame
        tf::Transform error = cur_gtrel_tf.inverse() * cur_odomrel_tf;
        // ROS_INFO_STREAM("cur_odomrel_tf x: " << cur_odomrel_tf.getOrigin().getX() << "\ty: " << cur_odomrel_tf.getOrigin().getY() << "\tz: " << cur_odomrel_tf.getOrigin().getZ());
        // ROS_INFO_STREAM("cur_gtrel_tf x: " << cur_gtrel_tf.getOrigin().getX() << "\ty: " << cur_gtrel_tf.getOrigin().getY() << "\tz: " << cur_gtrel_tf.getOrigin().getZ());

        double x, y, z, roll, pitch, yaw;
        ROS_INFO_STREAM("error x: " << error.getOrigin().getX() << "\ty: " << error.getOrigin().getY() << "\tz: " << error.getOrigin().getZ());
        tf::Matrix3x3(error.getRotation()).getRPY(roll, pitch, yaw);

        // calculate error in two different global frames, not in first pose frame
        // for example: carla global frame, gnss ENU frame
        // ROS_INFO_STREAM("error x: " << (cur_odom.pose.pose.position.x - first_odom_pose.pose.position.x) - (cur_gt_pose.pose.position.x - first_gt_pose.pose.position.x)
        //                 << "\ty: " << (cur_odom.pose.pose.position.y - first_odom_pose.pose.position.y) - (cur_gt_pose.pose.position.y - first_gt_pose.pose.position.y)
        //                 << "\tz: " << (cur_odom.pose.pose.position.z - first_odom_pose.pose.position.z) - (cur_gt_pose.pose.position.z - first_gt_pose.pose.position.z));

        // prepare for next search
        find_corresponding_frame = false;
    }
    // ROS_WARN("estimateQueue empty!");
}

void visualizeGT(){
    if (!first_gt_pose_set || !first_odom_pose_set)
    {
        return;
    }
    while (!vizgtQueue.empty())
    {
        geometry_msgs::PoseStamped cur_gt_pose = vizgtQueue.front();
        vizgtQueue.pop_front();

        tf::Transform gt_first_tf, cur_gt_tf, cur_gtrel_tf;
        tf::poseMsgToTF(first_gt_pose.pose, gt_first_tf);
        tf::poseMsgToTF(cur_gt_pose.pose, cur_gt_tf);
        cur_gtrel_tf = gt_first_tf.inverse() * cur_gt_tf;
    
        // calculate gt pose in relative frame(first odom)
        tf::Transform odom_first_tf, gt_in_odom_tf;
        tf::poseMsgToTF(first_odom_pose.pose, odom_first_tf);
        gt_in_odom_tf = odom_first_tf * cur_gtrel_tf;

        geometry_msgs::PoseStamped gt_in_odom;
        tf::poseTFToMsg(gt_in_odom_tf, gt_in_odom.pose);
        gt_in_odom.header = cur_gt_pose.header;
        pubGTPoseInOdomFrame.publish(gt_in_odom);

        nav_msgs::Odometry msg;
        msg.pose.pose = gt_in_odom.pose;
        msg.header = gt_in_odom.header;
        pubGTPoseInOdomFrame_nav.publish(msg);
    }
}

void threadMain(){
    ros::WallDuration dura(1);
    while(ros::ok()){
        calcError();
        dura.sleep();
    }
}

void visualMain(){
    ros::WallDuration dura(0.01);
    while(ros::ok()){
        visualizeGT();
        dura.sleep();
    }
}

int main(int argc, char** argv){

    ros::init(argc, argv, "traj_evaluation");
    ros::NodeHandle nh;

    nh.param<std::string>("version", version, "");

    if(nh.getParam("gt_topic", gt_topic) == false){
        ROS_ERROR_STREAM("please set gt_topic!");
        return -1;
    }    
    if(nh.getParam("traj_topic", traj_topic) == false){
        ROS_ERROR_STREAM("please set traj_topic!");
        return -1;
    }
    if(nh.getParam("folder_name", folder_name) == false){
        ROS_ERROR_STREAM("please set folder_name!");
        return -1;
    }


    subGTPose       = nh.subscribe<geometry_msgs::PoseStamped>(gt_topic, 2000, gtHandler, ros::TransportHints().tcpNoDelay());
    subEstimatePose = nh.subscribe<nav_msgs::Odometry>(traj_topic, 2000, odometryHandler, ros::TransportHints().tcpNoDelay());

    pubGTPoseInOdomFrame = nh.advertise<geometry_msgs::PoseStamped>("/gt_pose_in_odom_frame", 20);
    pubGTPoseInOdomFrame_nav = nh.advertise<nav_msgs::Odometry>("/gt_pose_in_odom_frame_nav_msg", 20);
    
    ROS_INFO("\033[1;32m----> trajectory evaluation Started.\033[0m");

    std::thread calcthread(threadMain);
    std::thread visualthread(visualMain);

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    calcthread.join();
    visualthread.join();

    return 0;
}