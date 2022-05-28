#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>

#include <Eigen/Geometry>

#include <thread>

// 使用carla真值位姿对激光雷达帧进行拼接获取地图
using namespace std;
string lidar_topic = "/carla/lidar1/point_cloud";
string pose_topic = "/ndt/current_pose";
string pcd_file_name = "/tmp/finalCloud.pcd";
geometry_msgs::PoseStamped gt_pose[100];
int pose_store_pointer = 0; // 存到这个指向的元素中
int pose_last_pointer = 0;
float voxel_size = 0.2;
geometry_msgs::Pose last_keyFrame_pose;  // 记录上一关键帧位置，根据距离判断要不要加入新点云
float keyFrame_dis = 5.0;
bool first_cloud = false;
using PointType = pcl::PointXYZ;
pcl::PointCloud<PointType>::Ptr global_map(new pcl::PointCloud<PointType>());
pcl::VoxelGrid<PointType> downSizeFilterGlobalMap; // for global map visualization
ros::Subscriber sub_cloud;
ros::Subscriber sub_pose;
ros::Publisher pub_vis_map;
ros::ServiceServer save_map;

float getDisBtwPoses2D(geometry_msgs::Pose& pose1, geometry_msgs::Pose& pose2){
    float temp = pow(pose1.position.x - pose2.position.x, 2) + pow(pose1.position.y - pose2.position.y, 2);
    return pow(temp, 0.5);
}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*msg, *cloud);
    // find same timestamp in poses
    uint32_t sec = msg->header.stamp.sec;
    uint32_t nsec = msg->header.stamp.nsec;
    bool useful_flag = false;
    int index = pose_last_pointer + 1;
    while (true) {
        if (gt_pose[index].header.stamp.sec == sec && gt_pose[index].header.stamp.nsec - nsec < fabs(5e6)){
            pose_last_pointer = index;
            useful_flag = true;
            break;
        }
        index = (index + 1) % 100;
        if(index == pose_store_pointer){
            useful_flag = false;
            break;
        }
    }
    if(!useful_flag){
        ROS_INFO_STREAM("cannot find the same timestamp for current cloud!");
        return;
    }
    // above is finding corresbonding timestamp of this cloud msg


    if (!first_cloud) {
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << gt_pose[pose_last_pointer].pose.position.x, gt_pose[pose_last_pointer].pose.position.y, gt_pose[pose_last_pointer].pose.position.z;
        transform.rotate(Eigen::Quaternionf(gt_pose[pose_last_pointer].pose.orientation.w,
                                            gt_pose[pose_last_pointer].pose.orientation.x,
                                            gt_pose[pose_last_pointer].pose.orientation.y,
                                            gt_pose[pose_last_pointer].pose.orientation.z));
        pcl::transformPointCloud(*cloud, *cloud, transform);
        *global_map += *cloud;
        last_keyFrame_pose = gt_pose[pose_last_pointer].pose;
        first_cloud = true;
    }

    if (getDisBtwPoses2D(last_keyFrame_pose, gt_pose[pose_last_pointer].pose) > 5.0) {
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << gt_pose[pose_last_pointer].pose.position.x, gt_pose[pose_last_pointer].pose.position.y, gt_pose[pose_last_pointer].pose.position.z;
        transform.rotate(Eigen::Quaternionf(gt_pose[pose_last_pointer].pose.orientation.w,
                                            gt_pose[pose_last_pointer].pose.orientation.x,
                                            gt_pose[pose_last_pointer].pose.orientation.y,
                                            gt_pose[pose_last_pointer].pose.orientation.z));
        pcl::transformPointCloud(*cloud, *cloud, transform);
        *global_map += *cloud;
        last_keyFrame_pose = gt_pose[pose_last_pointer].pose;
    }



    
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    gt_pose[pose_store_pointer] = *msg;
    pose_store_pointer++;
    pose_store_pointer %= 100;
}

void visualizeGlobalMapThread(){
    pcl::PointCloud<PointType>::Ptr vis_map(new pcl::PointCloud<PointType>());
    ros::Rate rate(0.2);
    while (ros::ok()){
        downSizeFilterGlobalMap.setInputCloud(global_map);
        downSizeFilterGlobalMap.filter(*vis_map);
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*vis_map, cloudMsgTemp);
        cloudMsgTemp.header.frame_id = "map";
        cloudMsgTemp.header.stamp = gt_pose[pose_last_pointer].header.stamp;
        pub_vis_map.publish(cloudMsgTemp);
        rate.sleep();
    }
}

bool saveMapCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response){
    ROS_INFO_STREAM("prepare to save the map into pcd file.");
    pcl::io::savePCDFileASCII(pcd_file_name, *global_map);
    ROS_INFO_STREAM("pcd file saved in: " << pcd_file_name);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "mappingCarla");
    ros::NodeHandle nh;

    sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 5, cloudCallback);
    sub_pose = nh.subscribe<geometry_msgs::PoseStamped>(pose_topic, 5, poseCallback);
    pub_vis_map = nh.advertise<sensor_msgs::PointCloud2>("vis_global_map", 5);
    save_map = nh.advertiseService("save_map", saveMapCallback);
    downSizeFilterGlobalMap.setLeafSize(0.4, 0.4, 0.4);

    thread visualizeMapThread(visualizeGlobalMapThread);

    while (ros::ok()) {
        ros::spin();
    }

    visualizeMapThread.join();

    return 0;

}
