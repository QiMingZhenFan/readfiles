#include <ndt_utility.h>

NdtMatching::NdtMatching(){
    nh.param<std::string>("localization/pointCloudTopic", pointCloudTopic, "points_raw");
    nh.param<std::string>("localization/imuOdomTopic", imuOdomTopic, "odometry/imu");
    nh.param<std::string>("localization/mapPath", map_path_, "");
    nh.param<std::string>("localization/ndtOdomTopic", ndtOdomTopic, "ndt/current_pose");

    nh.param<double>("localization/NDTEpsilon", ndt_epsilon_, 0.01);
    nh.param<double>("localization/NDTStepSize", ndt_step_size_, 0.05);
    nh.param<int>("localization/NDTMaxIteration", ndt_max_iteration_, 35);
    nh.param<double>("localization/NDTResolution", ndt_resolution_, 2.0);
    
    subImuOdom = nh.subscribe<nav_msgs::Odometry>(imuOdomTopic, 5, &NdtMatching::OdometryHandler, this, ros::TransportHints().tcpNoDelay());
    subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &NdtMatching::PointCloudHandler, this, ros::TransportHints().tcpNoDelay());
    sub_initial_pose_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &NdtMatching::InitialPoseHandler, this, ros::TransportHints().tcpNoDelay());

    pubNdtOdometry = nh.advertise<nav_msgs::Odometry>(ndtOdomTopic, 2000);
    pubGlobalMap = nh.advertise<sensor_msgs::PointCloud2>("global_map", 2000);
    
    global_map_.reset(new pcl::PointCloud<PointType>());
    global_map_downsample_.reset(new pcl::PointCloud<PointType>());
    current_pointcloud_.reset(new pcl::PointCloud<PointType>());
    initial_guess_ = Eigen::Matrix4f::Identity();

    ndt_.setTransformationEpsilon(ndt_epsilon_);
    ndt_.setStepSize(ndt_step_size_);
    ndt_.setMaximumIterations(ndt_max_iteration_);
    ndt_.setResolution(ndt_resolution_);
}


void NdtMatching::InitialPoseHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odom_msg){
    odom_timestamp_ = odom_msg->header.stamp.toSec();
    auto& t = odom_msg->pose.pose.position;
    auto& quaternion = odom_msg->pose.pose.orientation;
    Eigen::Quaternionf rotation(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    Eigen::Translation3f translation(t.x, t.y, t.z);

    // update initial guess
    initial_guess_.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    initial_guess_.block<3, 1>(0, 3) = translation.vector();
    initial_pose_set_ = true;
    initial_pose_used_ = false;
    ROS_INFO_STREAM("Initial pose has been set!");
}

void NdtMatching::OdometryHandler(const nav_msgs::Odometry::ConstPtr& odom_msg){
    // ROS_INFO_STREAM("i am in odometry handler!");

    if(!initial_pose_set_)
        return;
    odom_timestamp_ = odom_msg->header.stamp.toSec();
    auto& t = odom_msg->pose.pose.position;
    auto& quaternion = odom_msg->pose.pose.orientation;
    Eigen::Quaternionf rotation(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    Eigen::Translation3f translation(t.x, t.y, t.z);

    // update initial guess
    // initial_guess_.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    // initial_guess_.block<3, 1>(0, 3) = translation.vector();
}

void NdtMatching::PointCloudHandler(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg){
    if(!map_load_ || !initial_pose_set_)
        return;

    if (ndt_.getInputTarget() == nullptr) {
        ROS_WARN_STREAM_THROTTLE(1, "No MAP!");
        return;
    }
    ROS_DEBUG_STREAM("map size: " << ndt_.getInputTarget()->points.size());
    pcl::fromROSMsg(*pointcloud_msg, *current_pointcloud_);

    // downsample
    // ApproximateVoxelGrid -- faster but less precious
    // pcl::ApproximateVoxelGrid<PointType> filter;
    pcl::VoxelGrid<PointType> filter;
    filter.setLeafSize(0.2, 0.2, 0.2);
    filter.setInputCloud(current_pointcloud_);
    filter.filter(*current_pointcloud_);
    ROS_DEBUG_STREAM("cur point cloud size after downsample: " << current_pointcloud_->points.size());
    static Eigen::Matrix4f initial_value = Eigen::Matrix4f::Identity();
    if(!initial_pose_used_){
        initial_value = initial_guess_;
        initial_pose_used_ = true;
    }
        
    ndt_.setInputSource(current_pointcloud_);
    pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
    ndt_.align(*output_cloud, initial_value);
    ROS_DEBUG_STREAM("Normal Distributions Transform has converged: " << ndt_.hasConverged()
        << " score: " << ndt_.getFitnessScore() << " iterations: " << ndt_.getFinalNumIteration() << "/" << ndt_.getMaximumIterations());

    static Eigen::Matrix4f output = Eigen::Matrix4f::Identity();
    output = ndt_.getFinalTransformation();
    std::cout << "------------------------" << std::endl;
    std::cout << initial_value << std::endl;
    std::cout << output << std::endl;
    initial_value = output;

    // publish ndt odometry
    nav_msgs::Odometry ndt_odom;
    static unsigned int count = 0;
    ndt_odom.header.stamp = pointcloud_msg->header.stamp;
    ndt_odom.header.frame_id = "map";
    ndt_odom.header.seq = count++;
    ndt_odom.pose.pose.position.x = output(0,3);
    ndt_odom.pose.pose.position.y = output(1,3);
    ndt_odom.pose.pose.position.z = output(2,3);
    Eigen::Quaternionf quat(output.block<3, 3>(0, 0));
    ndt_odom.pose.pose.orientation.w = quat.w();
    ndt_odom.pose.pose.orientation.x = quat.x();
    ndt_odom.pose.pose.orientation.y = quat.y();
    ndt_odom.pose.pose.orientation.z = quat.z();
    pubNdtOdometry.publish(ndt_odom);

    static tf::TransformBroadcaster broadcaster;
    tf::Quaternion quad(quat.x(), quat.y(), quat.z(), quat.w());
    tf::Transform trans;
    trans.setOrigin(tf::Vector3(output(0,3), output(1,3), output(2,3))); 
    trans.setRotation(quad);
    broadcaster.sendTransform(tf::StampedTransform(trans, pointcloud_msg->header.stamp,"map","base_link"));
}

void NdtMatching::LoadGlobalMap(){
    if(map_path_.empty()){
        ROS_ERROR("Map path not set! cannot load global map in ndt module!");
        ros::shutdown();
    }
    if(pcl::io::loadPCDFile<PointType> (map_path_, *global_map_) == -1){
        ROS_ERROR("Cannot load global map in ndt module! map path: %s", map_path_.c_str());
        ros::shutdown();
    }
    ROS_INFO("Global Map has %ld points.", global_map_->size());

    // downsample
    pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud(global_map_);
    approximate_voxel_filter.filter(*global_map_downsample_);
    ROS_INFO("After downsample, global Map has %ld points.", global_map_->size());
    // global_map_downsample_.reset();
    // global_map_downsample_ = global_map_;


    ndt_.setInputTarget(global_map_);
    // pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
    // ndt_.align(*output_cloud, Eigen::Matrix4f::Identity());
    map_load_ = true;

    sensor_msgs::PointCloud2 pc_to_pub;
    pcl::toROSMsg(*global_map_downsample_, pc_to_pub);
    pc_to_pub.header.frame_id = "map";

    ros::Rate rate(0.2);
    bool have_published = false;
    while(ros::ok()){
        if(pubGlobalMap.getNumSubscribers() != 0){
            if(have_published == false){   // rviz too slow !!!!
                pc_to_pub.header.stamp = ros::Time::now();
                pubGlobalMap.publish(pc_to_pub);
            }
            have_published = true;
        }
        else{
            have_published = false;
        }
        rate.sleep();
    }
}