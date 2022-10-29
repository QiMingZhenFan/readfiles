#include "back_end.h"

#ifdef _OPENMP 
#include<omp.h>
#endif
namespace back_end{

const int kMaxIteration = 10;
const float kPairSelectRadius = 20.0; // m
const int kSubmapRadius = 5; // frame index
const double kGnssStdDev = 0.1; 
namespace{
double distance(geometry_msgs::Point& pos1, geometry_msgs::Point& pos2){
    double powdis = pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2) + pow(pos1.z - pos2.z, 2); 
    return pow(powdis, 0.5);
}

// index: the frame closest to target_time, but a little bit early
// index+1: the frame closest to target_time, but a little bit late
// return: if have frame newer than target_time
template<typename P>
bool FindNearstTwoFrame(const std::vector<P>& msg, double target_time, int& index){
    for(int i = index; i < msg.size() - 1; ++i){
        if(msg[i].header.stamp.toSec() <= target_time && target_time < msg[i+1].header.stamp.toSec()){
            index = i;
            return true;
        }
        if(msg[i].header.stamp.toSec() > target_time){
            return false;
        }
    }
    // all msgs timestamp < target_time
    return false;
}

void InterpolateTargetGnss(sensor_msgs::NavSatFix& target, 
                            const std_msgs::Header& header,
                            const sensor_msgs::NavSatFix& left,
                            const sensor_msgs::NavSatFix& right){
    target = left;
    target.header.stamp = header.stamp;
    double ratio = (header.stamp.toSec() - left.header.stamp.toSec()) / 
                    (right.header.stamp.toSec() - left.header.stamp.toSec());
    target.latitude += ratio * (right.latitude - left.latitude);
    target.longitude += ratio * (right.longitude - left.longitude);
    target.altitude += ratio * (right.altitude - left.altitude);
}

void InterpolateTargetPose(geometry_msgs::PoseStamped& target, 
                            const std_msgs::Header& header,
                            const geometry_msgs::PoseStamped& left,
                            const geometry_msgs::PoseStamped& right){
    target = left;
    target.header.stamp = header.stamp;
    double ratio = (header.stamp.toSec() - left.header.stamp.toSec()) / 
                    (right.header.stamp.toSec() - left.header.stamp.toSec());
    auto& vec4l = left.pose.orientation;
    Eigen::Quaterniond ql(vec4l.w, vec4l.x, vec4l.y, vec4l.z);
    auto& vec4r = right.pose.orientation;
    Eigen::Quaterniond qr(vec4r.w, vec4r.x, vec4r.y, vec4r.z); 
    Eigen::Quaterniond qres = ql.slerp(ratio, qr);

    target.pose.position.x += ratio * (right.pose.position.x - left.pose.position.x);
    target.pose.position.y += ratio * (right.pose.position.y - left.pose.position.y);
    target.pose.position.z += ratio * (right.pose.position.z - left.pose.position.z);
    target.pose.orientation.w = qres.w();
    target.pose.orientation.x = qres.x();
    target.pose.orientation.y = qres.y();
    target.pose.orientation.z = qres.z();
}

void WritePoseToFile(const std::string& file_path_name, const std::unordered_map<int, Node>& nodes){
    // in TUM format
    std::ofstream outFile(file_path_name, std::ios::out);
    if(!outFile){
        ROS_ERROR_STREAM("Cannot write file : " << file_path_name);
        return;
    }
    for(int i = 0; i <nodes.size(); ++i){
        auto& node = nodes.at(i);
        outFile << node.time_stamp.toSec()
                << ' ' << node.opt_pose.position.x()
                << ' ' << node.opt_pose.position.y()
                << ' ' << node.opt_pose.position.z()
                << ' ' << node.opt_pose.rotation.x()
                << ' ' << node.opt_pose.rotation.y()
                << ' ' << node.opt_pose.rotation.z()
                << ' ' << node.opt_pose.rotation.w()
                << std::endl;
    }
    outFile.close();
}

void WriteFinalPointCloudToPCDFile(const std::string& file_path_name, const std::unordered_map<int, Node>& nodes, bool original){
    pcl::PointCloud<PointType>::Ptr total_map(new pcl::PointCloud<PointType>());
    for(const auto& [id, node] : nodes){
        pcl::PointCloud<PointType>::Ptr temp_map(new pcl::PointCloud<PointType>());
        if(original)
            pcl::transformPointCloud(*node.cloud, *temp_map, node.pose.CalcTransformMat());
        else
            pcl::transformPointCloud(*node.cloud, *temp_map, node.opt_pose.CalcTransformMat());
        *total_map += *temp_map;
    }
    // downsample
    ROS_INFO_STREAM("Downsampling, pre points: " << total_map->points.size() << ".");
    pcl::PointCloud<PointType>::Ptr total_map_downsampled(new pcl::PointCloud<PointType>());
    double voxel_leaf_size = 0.2;
    pcl::VoxelGrid<PointType> voxel;
    voxel.setInputCloud(total_map);
    voxel.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel.filter(*total_map_downsampled);
    ROS_INFO_STREAM("Downsampled, aft points: " << total_map_downsampled->points.size() << ".");
    ROS_INFO_STREAM("Saving PCD file...");

    pcl::io::savePCDFileASCII(file_path_name, *total_map_downsampled);
    ROS_INFO_STREAM("PCD file saved!");
}

void WriteSubmapPointCloudToPCDFile(const std::string& file_path, const Node& node, int id){
    pcl::PointCloud<PointType>::Ptr temp_map(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*node.cloud, *temp_map, node.pose.CalcTransformMat());
    pcl::io::savePCDFileASCII(file_path + std::to_string(id) + ".pcd", *(node.cloud));
}
} // namespace


void BackEnd::Init(){
    param_use_robust_kernel_ = true;
    param_use_gnss_ = false;
    lidar_frame_nums_ = 0;
    node_nums_ = 0;
    param_output_file_path = "/tmp/opt_pose.csv";
    param_output_pcdfile_path = "/tmp/total_map.pcd";

    // check if topic is empty
    if(param_gnss_topic_.empty() || param_pose_topic_.empty() || param_lidar_topic_.empty()){
        ROS_ERROR_STREAM("topic name is empty, please set topic name first!");
        ros::shutdown();
    }
    
    // set omp threads
    int thread = 1;
    int total_threads = thread;
#ifdef _OPENMP 
    total_threads = omp_get_max_threads();
    thread = total_threads > 2? total_threads - 1: total_threads;
    omp_set_num_threads(thread);
#endif
    ROS_INFO_STREAM("Total threads: " << total_threads << ", use threads: " << thread << ".");

    // for quick push
    point_clouds_.reserve(1000);
    gnss_.reserve(1000);
    odometry_poses_.reserve(1000);

    pose_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

    optimizer.setVerbose(false);
    // linearSolver = g2o::make_unique<
    //     g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
    linearSolver = g2o::make_unique<
        g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));


    optimizer.setAlgorithm(solver);
}

// read bag file to load poses, gnss readings and lidar points
void BackEnd::ReadDataFromBag(std::vector<sensor_msgs::PointCloud2>& point_clouds,\
                                std::vector<sensor_msgs::NavSatFix>& gnss,\
                                std::vector<geometry_msgs::PoseStamped>& odometry_poses){
    rosbag::Bag bag;    

    bag.open(param_bag_full_path_, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(param_lidar_topic_);
    topics.push_back(param_pose_topic_);
    topics.push_back(param_gnss_topic_);

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    // rosbag::View view(bag, rosbag::TopicQuery(topics),ros::Time(1657809600), ros::Time(1657809650));

    ros::Time bag_begin_time = view.getBeginTime();
    ros::Time bag_end_time = view.getEndTime();
    assert((bag_end_time-bag_begin_time).toSec() > 1.0 && "bag time < 1.0s!");
    ROS_INFO_STREAM("ROS bag time: " << (bag_end_time-bag_begin_time).toSec() << "(s)");

    // TODO: wonder if this method make sure datas follow the timing order
    for(const rosbag::MessageInstance& m:view){
    if (m.getTopic() == param_lidar_topic_ || ("/" + m.getTopic() == param_lidar_topic_))
    {
    sensor_msgs::PointCloud2::Ptr message = m.instantiate<sensor_msgs::PointCloud2>();
    if (message != NULL)
    point_clouds.emplace_back(*message);
    }
    if (m.getTopic() == param_pose_topic_ || ("/" + m.getTopic() == param_pose_topic_))
    {
    // TODO: determine which kind of message to use here
    geometry_msgs::PoseStamped::Ptr message0 = m.instantiate<geometry_msgs::PoseStamped>();
    if (message0 != NULL){
    odometry_poses.emplace_back(*message0);
    continue;
    }
    nav_msgs::Odometry::Ptr message1 = m.instantiate<nav_msgs::Odometry>();
    if (message1 != NULL){
    odometry_poses.emplace_back();
    auto& cur_posestamped = odometry_poses.back();
    cur_posestamped.header = message1->header;
    cur_posestamped.pose = message1->pose.pose;
    continue;
    }
    }
    if (m.getTopic() == param_gnss_topic_ || ("/" + m.getTopic() == param_gnss_topic_))
    {
    sensor_msgs::NavSatFix::Ptr message = m.instantiate<sensor_msgs::NavSatFix>();
    if (message != NULL){
    gnss.emplace_back(*message);
    }
    }
    }
    bag.close();
    }

void BackEnd::AlignTimeStamp(std::vector<sensor_msgs::PointCloud2>& point_clouds,
                                std::vector<sensor_msgs::NavSatFix>& gnss,
                                std::vector<geometry_msgs::PoseStamped>& odometry_poses){
    // temporarily store the output
    std::vector<sensor_msgs::PointCloud2> temp_pc;
    std::vector<sensor_msgs::NavSatFix> temp_gnss;
    std::vector<geometry_msgs::PoseStamped> temp_op;

    int index_gnss = 0, index_op = 0;
    for(auto& pc : point_clouds){
        double pc_time = pc.header.stamp.toSec();
        // if align failed, we discard this lidar frame
        // interpolation here, [index, index+1]
        if(param_use_gnss_){
            if(FindNearstTwoFrame(gnss, pc_time, index_gnss) 
                && FindNearstTwoFrame(odometry_poses, pc_time, index_op)){
                temp_pc.emplace_back(pc);
                // form new GNSS
                temp_gnss.emplace_back();
                auto& target_gnss = temp_gnss.back();
                InterpolateTargetGnss(target_gnss, pc.header, gnss[index_gnss], gnss[index_gnss+1]);
                // form new odometry pose
                temp_op.emplace_back();
                auto& target_pose = temp_op.back();
                InterpolateTargetPose(target_pose, pc.header, odometry_poses[index_op], odometry_poses[index_op+1]);
            }
        }
        else{
            if(FindNearstTwoFrame(odometry_poses, pc_time, index_op)){
                temp_pc.emplace_back(pc);
                // form new odometry pose
                temp_op.emplace_back();
                auto& target_pose = temp_op.back();
                InterpolateTargetPose(target_pose, pc.header, odometry_poses[index_op], odometry_poses[index_op+1]);
                // ROS_INFO("timestamp of pc: %f, and odom front: %f, and odom back: %f.", 
                //         pc_time, 
                //         odometry_poses[index_op].header.stamp.toSec(),
                //         odometry_poses[index_op+1].header.stamp.toSec());
            }
        }
    }
    // TODO: find a fast way to swap 2 vectors
    point_clouds.swap(temp_pc);
    gnss.swap(temp_gnss);
    odometry_poses.swap(temp_op);
}

void BackEnd::CollectData(){
    ReadDataFromBag(point_clouds_, gnss_, odometry_poses_);
    if(param_use_gnss_ && gnss_.size() == 0){
        ROS_ERROR("set use gnss but no gnss data!");
        ros::shutdown();
    }
    ROS_INFO_STREAM("Origin vector size, point_clouds: " << point_clouds_.size()
                                            << ", gnss: " << gnss_.size() 
                                            << ", odometry: " << odometry_poses_.size());

    AlignTimeStamp(point_clouds_, gnss_, odometry_poses_);
    ROS_INFO_STREAM("Vector size after timestamp alignment, point_clouds: " << point_clouds_.size()
                                                << ", gnss: " << gnss_.size() 
                                                << ", odometry: " << odometry_poses_.size());
    if(param_use_gnss_)
        assert(point_clouds_.size() == gnss_.size() && point_clouds_.size() == odometry_poses_.size());    
    else
        assert(point_clouds_.size() == odometry_poses_.size());    

    auto ConstructSubmap = [this](int left, int mid, int right, Node& node){
        left = left < 0? 0: left;
        right = right > point_clouds_.size()-1? point_clouds_.size()-1: right;
        Eigen::Transform<double,3,Eigen::Isometry>  trans_mid = node.pose.position * node.pose.rotation;
        pcl::fromROSMsg(point_clouds_[mid], *node.cloud);
        for(int i = left; i <= right; ++i){
            if(i == mid){
                continue;
            }
            else{
                Eigen::Transform<double,3,Eigen::Isometry>  trans_cur;
                trans_cur.translation() << odometry_poses_[i].pose.position.x, 
                                            odometry_poses_[i].pose.position.y,
                                            odometry_poses_[i].pose.position.z;
                trans_cur.matrix().block<3,3>(0,0) = Eigen::Quaterniond(odometry_poses_[i].pose.orientation.w,
                                                                        odometry_poses_[i].pose.orientation.x,
                                                                        odometry_poses_[i].pose.orientation.y,
                                                                        odometry_poses_[i].pose.orientation.z).toRotationMatrix();
                Eigen::Transform<double,3,Eigen::Isometry>  trans_mid_cur = trans_mid.inverse() * trans_cur;
                pcl::PointCloud<PointType> point_cloud_cur;
                pcl::fromROSMsg(point_clouds_[i], point_cloud_cur);
                pcl::transformPointCloud(point_cloud_cur, point_cloud_cur, trans_mid_cur.matrix().cast<float>());
                *node.cloud += point_cloud_cur;
            }
        }
    };

    for(int i = 0; i < point_clouds_.size(); ++i){
        // TODO: only sample in index space here, should take distance and time interval into account
        if(i != 0 && i % kSubmapRadius == 0){
            // 1. construct nodes here
            int node_index = nodes.size();
            nodes[node_index] = Node(node_index, point_clouds_[i].header.stamp);
            auto& temp_node = nodes[node_index];
            // 2. fill pose
            temp_node.pose.position.translation() << odometry_poses_[i].pose.position.x, 
                                                    odometry_poses_[i].pose.position.y,
                                                    odometry_poses_[i].pose.position.z;
            temp_node.pose.rotation = Eigen::Quaterniond(odometry_poses_[i].pose.orientation.w,
                                                        odometry_poses_[i].pose.orientation.x,
                                                        odometry_poses_[i].pose.orientation.y,
                                                        odometry_poses_[i].pose.orientation.z);
            // 3. fill gnss pose and information
            // change gps readings in wsg84 to utm
            if(param_use_gnss_){
                geographic_msgs::GeoPoint geo_point = geodesy::toMsg(gnss_[i]);
                geodesy::UTMPoint utm;
                geodesy::fromMsg(geo_point, utm);
                temp_node.gnss_pose.position.translation() << utm.easting, utm.northing, utm.altitude;
                temp_node.gnss_information = Eigen::Matrix6d::Identity() * (1 / kGnssStdDev);
            }

            // 4. fill cloud
            ConstructSubmap(i - kSubmapRadius, i, i + kSubmapRadius, temp_node);
            pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>());
            double voxel_leaf_size = 0.2;
            pcl::VoxelGrid<PointType> voxel;
            voxel.setInputCloud(temp_node.cloud);
            voxel.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
            voxel.filter(*cloud_filtered);
            temp_node.cloud = cloud_filtered;

            // 5. put node pose into pose_cloud for kdTree find KNN
            pcl::PointXYZI pose_point;
            pose_point.x = temp_node.pose.position.translation().x();
            pose_point.y = temp_node.pose.position.translation().y();
            pose_point.z = temp_node.pose.position.translation().z();
            pose_point.intensity = node_index;
            pose_cloud->push_back(pose_point);

            WriteSubmapPointCloudToPCDFile("/code/maps/", temp_node, node_index);
            ROS_INFO_STREAM("in constructing submap, node id: " << node_index  << ", submap points: " << temp_node.cloud->size());
        }
    }

    // set node_nums_
    node_nums_ = nodes.size();
    lidar_frame_nums_ = point_clouds_.size();
}

bool BackEnd::AddPair(const int node1_id, const int node2_id){
    // must ensure front id < second id
    int less_id = std::min(node1_id, node2_id);
    int larger_id = std::max(node1_id, node2_id);
    if(less_id == larger_id){
        ROS_WARN_STREAM_COND(less_id == larger_id, "pairs have same id, id: " << less_id << "!");
        return false;
    }
    if(pairs.count(less_id) != 0 && pairs[less_id].count(larger_id) != 0){
        ROS_WARN_STREAM("pair already exist!");
        return false;
    }
    pairs[less_id].insert(larger_id); 
    // TODO: make sure every node was constrainted
    // node_if_constrainted[less_id] = true;
    // node_if_constrainted[larger_id] = true;
    return true;
}

void BackEnd::DividePairs(){
    int pair_nums = 0;
    kdtree.setInputCloud(pose_cloud);
    // TODO: this part can be parallel
    for(const auto& point : pose_cloud->points){
        std::vector<int> pointIdxSearch;
        std::vector<float> pointSquaredDistance;
        if (kdtree.radiusSearch(point, kPairSelectRadius, pointIdxSearch, pointSquaredDistance) > 0){
            for(const auto& id : pointIdxSearch){
                if(AddPair(point.intensity, pose_cloud->points[id].intensity)){
                    ++pair_nums;
                }
            }
        }
    } 
    // TODO: check all node have interframe constraint before matching
    ROS_INFO_STREAM("total pairs: " << pair_nums);
}


void BackEnd::PairMatching(const int node1_id, const int node2_id){
    if(constraints.count(node1_id) != 0 && constraints[node1_id].count(node2_id) != 0){
        ROS_WARN("pair already matched!");
        return;
    }
    // for now, it's scan-to-scan matching
    const Node& node1 = nodes[node1_id];
    const Node& node2 = nodes[node2_id];
    // makeShared() is a deep copy
    pcl::PointCloud<PointType>::ConstPtr tar_cloud = node1.cloud;
    pcl::PointCloud<PointType>::ConstPtr src_cloud = node2.cloud;
    pcl::PointCloud<PointType> aligned_cloud;

    // ROS_INFO_STREAM("calculate delta transformation.");
    Eigen::Transform<double,3,Eigen::Isometry>  trans1 = node1.pose.position * node1.pose.rotation;
    Eigen::Transform<double,3,Eigen::Isometry>  trans2 = node2.pose.position * node2.pose.rotation;
    Eigen::Transform<double,3,Eigen::Isometry>  delta_trans = trans1.inverse() * trans2;

    // ros::WallTime bef = ros::WallTime::now();
    // ROS_INFO_STREAM("start icp matching.");
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
    gicp.setInputSource(src_cloud);
    gicp.setInputTarget(tar_cloud);
    gicp.setMaximumIterations(kMaxIteration);
    gicp.align(aligned_cloud, delta_trans.matrix().cast<float>());
    // ros::WallTime aft = ros::WallTime::now();
    // ROS_INFO_STREAM("icp consume time:" <<  (aft - bef).toSec() << ".");

    // TODO: add more metrics to judge the outcome
    // add constarints here
    if(gicp.hasConverged() && gicp.getFitnessScore() < 2){
        Constraint temp;
        temp.transform_1_2 = gicp.getFinalTransformation();
        temp.node1_id = node1.index;
        temp.node2_id = node2.index;
        temp.fitness_score = gicp.getFitnessScore();
        # pragma omp critical
        {
            constraints[node1.index][node2.index] = temp;
            ROS_INFO_STREAM("pair accepted: " << node1.index << " <---> " << node2.index <<
                            " with fitness score: " << temp.fitness_score <<".");
        }
        return;
    }
    # pragma omp critical
    {
        ROS_WARN_STREAM("reject this pair: " << node1.index << " <---> " << node2.index <<
                        " with fitness score: " << gicp.getFitnessScore() <<".");
    }
}

void BackEnd::AddConstraintToGraph(){
    bool has_fixed_node = false;
    // add vertex
    for(const auto& [id, node] : nodes){
        g2o::VertexSE3* v = new g2o::VertexSE3;
        v->setId(id);
        Eigen::Isometry3d t;
        t.setIdentity();
        t.translation() = node.pose.position.vector();
        t.matrix().block<3,3>(0, 0) = node.pose.rotation.toRotationMatrix();
        v->setEstimate(t);
        if(!param_use_gnss_){
            // assume the first node marked as id:0
            if (id == 0)
            {
                v->setFixed(true);
                has_fixed_node = true;
            }
        }
        if(param_use_gnss_){
            // add gnss unary edge to every node
            g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ());
            edge->setMeasurement(node.gnss_pose.position.vector());
            edge->setInformation(node.gnss_information.block<3,3>(0,0));
            edge->vertices()[0] = v;
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            edge->setRobustKernel(rk);
            optimizer.addEdge(edge);
            edges_gnss.push_back(edge);
        }
        optimizer.addVertex(v);
        vertices[id] = v;
    }

    // add edge
    for(const auto& [id1, temp_map] : constraints){
        for(const auto& [id2, cur_constraint] : temp_map){
            Eigen::Vector6d diagonal_vec;
            diagonal_vec << cur_constraint.fitness_score, cur_constraint.fitness_score,
                            cur_constraint.fitness_score, cur_constraint.fitness_score,
                            cur_constraint.fitness_score, cur_constraint.fitness_score;
            // TODO: test if this way to set information is stable
            Eigen::Matrix<double, 6, 6> information = diagonal_vec.asDiagonal().inverse();
            // information.block<3, 3>(0, 0) = transNoise.inverse();
            // information.block<3, 3>(3, 3) = rotNoise.inverse();
            g2o::VertexSE3* prev = vertices[id1];
            g2o::VertexSE3* cur = vertices[id2];
            Eigen::Isometry3d t(cur_constraint.transform_1_2.cast<double>());
            g2o::EdgeSE3* e = new g2o::EdgeSE3;
            e->setVertex(0, prev);
            e->setVertex(1, cur);
            e->setMeasurement(t);
            e->setInformation(information);
            if (param_use_robust_kernel_) {
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
            }
            optimizer.addEdge(e);
            edges.push_back(e);
        }
    }

    if(!has_fixed_node && !param_use_gnss_){
        ROS_ERROR( "make sure have at least one node fixed when no gnss data! shutdown this node.");
        ros::shutdown();
    }

    ROS_INFO("Current graph has %lu nodes and %lu edges!", vertices.size(), edges.size());
}

void BackEnd::Run(){
    ros::WallTime bef(ros::WallTime::now());
    Init();
    ROS_INFO("**************  collect data  **************");
    CollectData();
    
    ROS_INFO("**************  divide pairs  **************");
    DividePairs();

    ROS_INFO("**************  perform pair matching  **************");
    // hack, for omp parallel
    std::vector<std::pair<int, int>> pairs_vec;
    for (auto& [id1, id2s] : pairs) {
        for(auto& id2:id2s){
            pairs_vec.emplace_back(std::make_pair(id1, id2));
        }
    }
    int pair_calculated_num = 0;
    # pragma omp parallel for schedule(dynamic)
    for(int i = 0; i < pairs_vec.size(); ++i){
        int id1 = pairs_vec[i].first;
        int id2 = pairs_vec[i].second;
        PairMatching(id1, id2);
        # pragma omp critical
        {
            pair_calculated_num++;
            ROS_INFO("pair calculated num: %d/%ld.", pair_calculated_num, pairs_vec.size());
        }
    }

    ROS_INFO("**************  add constraints to graph  **************");
    AddConstraintToGraph();

    ROS_INFO("**************  start optimization  **************");
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(50);
    
    ROS_INFO("**************  finish optimization  **************");
    // write optimized pose into Node struct
    WriteFinalPointCloudToPCDFile("/tmp/orig_map.pcd", nodes, true);
    for(auto& [id, node] : nodes){
        double xyzqxyzw[7];
        vertices[id]->getEstimateData(xyzqxyzw);
        node.opt_pose.position.translation() << xyzqxyzw[0], xyzqxyzw[1], xyzqxyzw[2];
        node.opt_pose.rotation = Eigen::Quaterniond(xyzqxyzw[6], xyzqxyzw[3], xyzqxyzw[4], xyzqxyzw[5]);
    }

    ROS_INFO("**************  write data  **************");
    ROS_INFO_STREAM("write data to: " << param_output_file_path);
    WritePoseToFile(param_output_file_path, nodes);
    WriteFinalPointCloudToPCDFile(param_output_pcdfile_path, nodes, false);
    ros::WallTime aft(ros::WallTime::now());
    ROS_INFO_STREAM("total time consuming: " << (aft - bef).toSec() << "s.");

}
} // namespace back_end
