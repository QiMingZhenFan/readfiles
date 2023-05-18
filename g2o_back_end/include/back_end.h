#ifndef G2O_BACK_END_INCLUDE_BACK_END_H_
#define G2O_BACK_END_INCLUDE_BACK_END_H_

#include <iostream>
#include <fstream>

#include <sstream>
#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>


#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
// #include <pcl/kdtree/kdtree_flann.h> // cause lz4 error in compile
#include <pcl/kdtree/kdtree.h> 
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/conditional_removal.h>

#include "g2o/config.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/solver.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
// #include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/vertex_se3.h"
// hpps below are adapted from hdl-graph-slam
// #include <g2o/edge_se3_plane.hpp>
// #include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
// #include <g2o/edge_se3_priorvec.hpp>
// #include <g2o/edge_se3_priorquat.hpp>
// #include <g2o/edge_plane_prior.hpp>
// #include <g2o/edge_plane_identity.hpp>
// #include <g2o/edge_plane_parallel.hpp>
// #include <g2o/robust_kernel_io.hpp>

#include "ros/ros.h"
#include "ros/duration.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "tf/transform_datatypes.h"
#include "geographic_msgs/GeoPoint.h"
#include <geographic_msgs/GeoPose.h>
#include <geodesy/wgs84.h>
#include <geodesy/utm.h>

namespace Eigen {
using Matrix6d = ::Eigen::Matrix<double, 6, 6>;
using Vector6d = ::Eigen::Matrix<double, 6, 1>;
using Vector7d = ::Eigen::Matrix<double, 7, 1>;
inline Eigen::Matrix3d Skew(const Eigen::Vector3d& x) {
  // return x.cross(Eigen::Matrix3d::Identity());
  Eigen::Matrix3d x_hat;
  x_hat << 0, -x(2), x(1), x(2), 0, -x(0), -x(1), x(0), 0;
  return x_hat;
}
}  // namespace Eigen

namespace back_end{

using PointType = pcl::PointXYZ;

// TODO: use different functions according to the odometry data type 
enum OdometryDataType{
    kGeometryMsgPose = 0,
    kNavMsgOdometry = 1,
};

struct Pose
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Translation3d position;
    Eigen::Quaterniond rotation;
    Eigen::Matrix4f CalcTransformMat() const{
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3,3>(0,0) = rotation.toRotationMatrix().template cast<float>();
        transform.block<3,1>(0,3) = position.vector().template cast<float>();
        return transform;
    }
};

struct Node{
    int index;
    ros::Time time_stamp;
    pcl::PointCloud<PointType>::Ptr cloud;
    struct Pose pose, gnss_pose, opt_pose;
    Eigen::Matrix6d gnss_information;
    Node() {
        cloud.reset(new pcl::PointCloud<PointType>());
    };
    Node(int id, ros::Time time):index(id), time_stamp(time) {
        cloud.reset(new pcl::PointCloud<PointType>());
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Constraint{
    int  node1_id;
    int  node2_id;
    Eigen::Matrix4f transform_1_2; 
    double fitness_score;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class BackEnd{
    public:
    BackEnd();
    BackEnd(std::string bag_full_path, std::string lidar_topic, 
                        std::string pose_topic, std::string gnss_topic = "None") : 
                        param_bag_full_path_(bag_full_path),
                        param_lidar_topic_(lidar_topic),
                        param_pose_topic_(pose_topic),
                        param_gnss_topic_(gnss_topic) {};
    BackEnd(const BackEnd&) = delete;
    BackEnd& operator=(const BackEnd&) = delete;

    void Init();
    void ReadDataFromBag(std::vector<sensor_msgs::PointCloud2>& point_clouds,
                                                        std::vector<sensor_msgs::NavSatFix>& gnss,
                                                        std::vector<geometry_msgs::PoseStamped>& odometry_poses);
    void AlignTimeStamp(std::vector<sensor_msgs::PointCloud2>& point_clouds,
                                                    std::vector<sensor_msgs::NavSatFix>& gnss,
                                                    std::vector<geometry_msgs::PoseStamped>& odometry_poses);
    void CollectData(); 
    bool AddPair(const int node1_id, const int node2_id);
    void DividePairs();
    void PairMatching(const int node1_id, const int node2_id);
    void AddConstraintToGraph();
    void Run();

    private:
    int lidar_frame_nums_;
    int node_nums_; // should equal to lidar frames, if no key frame selected strategy
    bool param_use_robust_kernel_;
    bool param_use_gnss_;
    const std::string param_bag_full_path_;
    const std::string param_lidar_topic_;
    const std::string param_pose_topic_;
    const std::string param_gnss_topic_;
    std::string param_output_file_path;
    std::string param_output_pcdfile_path;

    std::unordered_map<int, Node> nodes;
    std::unordered_map<int, std::unordered_set<int>> pairs;
    // node1, node2, transform_between
    std::unordered_map<int, std::unordered_map<int, Constraint>> constraints;
    // for radius search, intensity used to store node index
    pcl::PointCloud<pcl::PointXYZI>::Ptr pose_cloud;
    pcl::search::KdTree<pcl::PointXYZI> kdtree;

    std::vector<sensor_msgs::PointCloud2> point_clouds_;
    std::vector<sensor_msgs::NavSatFix> gnss_;
    std::vector<geometry_msgs::PoseStamped> odometry_poses_;

    std::unordered_map<int, g2o::VertexSE3*> vertices;
    std::vector<g2o::EdgeSE3*> edges;
    std::vector<g2o::EdgeSE3PriorXYZ*> edges_gnss;
    // std::vector<int> node_if_constrainted;
    g2o::SparseOptimizer optimizer;
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    g2o::OptimizationAlgorithmLevenberg* solver;
};

} // namespace back_end


#endif // G2O_BACK_END_INCLUDE_BACK_END_H_