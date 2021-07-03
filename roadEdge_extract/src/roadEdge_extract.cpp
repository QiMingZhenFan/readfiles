#include "pcl/ModelCoefficients.h"
#include "pcl/PointIndices.h"
#include "pcl/common/io.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/sample_consensus/sac_model_plane.h"
#include "pcl/sample_consensus/ransac.h"
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/impl/point_types.hpp"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/pcl_base.h"
#include "pcl/surface/on_nurbs/fitting_surface_pdm.h"
#include "pcl/surface/on_nurbs/nurbs_data.h"
#include "ros/ros.h"
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <limits>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/project_inliers.h>

#include "pcl/surface/on_nurbs/fitting_surface_tdm.h"
#include "pcl/surface/on_nurbs/fitting_curve_2d_asdm.h"
#include "pcl/surface/on_nurbs/triangulation.h"

#include <vector>
#include <mutex>

#define PointType pcl::PointXYZI
# define M_PI		3.14159265358979323846	/* pi */

pcl::PointCloud<PointType> source_points;

int N_SCANS = 64;
float top_angle = 16.6;
float bottom_angle = -16.6;
float angle_resolution = (top_angle - bottom_angle) / N_SCANS;
std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
std::vector<std::vector<float> > centralHeights(N_SCANS * 2);     // 保留车前高度和车后高度，偶数为前，奇数为后

pcl::PointCloud<PointType> points_visualization;
std_msgs::Header msgHeader;

ros::Publisher pubRoadEdge;
ros::Publisher pubTempViz;
ros::Publisher pubTempViz2;

void pub_temp_viz(pcl::PointCloud<PointType> &points, ros::Publisher &pub = pubTempViz){
    sensor_msgs::PointCloud2 laserCloudMsg;
    pcl::toROSMsg(points, laserCloudMsg);
    laserCloudMsg.header.stamp = msgHeader.stamp;
    laserCloudMsg.header.frame_id = msgHeader.frame_id;
    pub.publish(laserCloudMsg);
}

float distance_to_plane(const PointType &point, float A, float B, float C, float D){
    return fabs(point.x * A + point.y * B + point.z * C + D) / std::sqrt(A*A + B*B + C*C);
}

void PointCloud2Vector3d (pcl::PointCloud<PointType>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data){
    for (unsigned i = 0; i < cloud->size (); i++)
    {
        PointType &p = cloud->at (i);
        if (!std::isnan (p.x) && !std::isnan (p.y) && !std::isnan (p.z))
        data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
    }
}

void PointCloud2Vector2d (pcl::PointCloud<PointType>::Ptr cloud, pcl::on_nurbs::vector_vec2d &data){
    // 直接截取z轴，最后使用平面方程恢复好了
    for (unsigned i = 0; i < cloud->size (); i++)
    {
        PointType &p = cloud->at (i);
        if (!pcl_isnan (p.x) && !pcl_isnan (p.y))
        data.push_back (Eigen::Vector2d (p.x, p.y));
    }
}
void PointCloud2Vector2d (pcl::PointCloud<PointType>::Ptr cloud, std::vector<float> &x, std::vector<float> &y){
    // 直接截取z轴，最后使用平面方程恢复好了
    for (unsigned i = 0; i < cloud->size (); i++)
    {
        PointType &p = cloud->at (i);
        if (!pcl_isnan (p.x) && !pcl_isnan (p.y)){
            x.push_back(p.x);
            y.push_back(p.y);
        }
    }
}

/////////////////////////////////////////
// 函数声明
Eigen::VectorXf FitterLeastSquareMethod(pcl::PointCloud<PointType>::Ptr cloud_in, uint8_t orders = 3);
void restorePointsFrom2D(pcl::PointCloud<PointType>::Ptr cloud, float X_min, float X_max, const Eigen::VectorXf& coefficients_curve, const Eigen::VectorXf &coefficients_plane, int resolution = 50);
void getMaxAndMin(pcl::PointCloud<PointType>::Ptr cloud, float &X_min, float &X_max);

// 投影点云到平面上
pcl::PointCloud<PointType>::Ptr ProjectPointCloudToPlane(pcl::PointCloud<PointType>::ConstPtr cloud, pcl::ModelCoefficientsPtr coefficients){
    pcl::PointCloud<PointType>::Ptr aftProjections (new pcl::PointCloud<PointType>);
    pcl::ProjectInliers<PointType> project;
    project.setModelType(pcl::SACMODEL_PLANE);
    project.setInputCloud(cloud);
    project.setModelCoefficients(coefficients);
    project.filter(*aftProjections);
    return aftProjections;
}

pcl::PointCloud<PointType>::Ptr fitting_curve_to_2d_spline(pcl::PointCloud<PointType>::Ptr cloud, pcl::ModelCoefficientsPtr coefficients){
    if (coefficients->values.size() != 4){
        ROS_ERROR("plane model should have 4 values!");
        return nullptr;
    }

    pcl::PointCloud<PointType>::Ptr aftProjections = ProjectPointCloudToPlane(cloud, coefficients);

    // 使用no_nurbs进行2d curve拟合
    pcl::on_nurbs::NurbsDataCurve2d data;
    PointCloud2Vector2d(aftProjections, data.interior);

    unsigned order(3);
    unsigned n_control_points(10);
    ON_NurbsCurve curve = pcl::on_nurbs::FittingCurve2dASDM::initNurbsCurve2D (order, data.interior, n_control_points); 
    pcl::on_nurbs::FittingCurve2dASDM::Parameter curve_params;
    curve_params.smoothness = 0.000001;
    curve_params.rScale = 1.0;
    pcl::on_nurbs::FittingCurve2dASDM fit (&data, curve); //初始化曲线拟合对象 
    fit.assemble (curve_params);                          //装配曲线参数
    fit.solve ();                                         //拟合
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, temp_cloud, 8);
    aftProjections->clear();
    for (auto point : *temp_cloud){
        pcl::PointXYZI temp_point;
        temp_point.x = point.x;
        temp_point.y = point.y;
        temp_point.z =  -(coefficients->values[0] * point.x + coefficients->values[1] * point.y + coefficients->values[3]) / coefficients->values[2];
        temp_point.intensity = std::rand();
        aftProjections->points.push_back(temp_point);
    }
    return aftProjections;
}


// TODO: 1.转弯时候并不鲁棒，这样按atan2()切割左半边右半边在转弯处会出问题
// TODO: 2.中间盲区太大，前后端有点的区域相对于盲区太小，导致拟合的曲线在盲区会比较奇怪
// TODO: 3.最终筛选出的点还是会有错误的情况，比如一下子一个平面、树根部的点等，并且可用点相对较少
std::mutex mtx_pointcloud, mtx_order;
void roadEdgeExtract(){
    if(mtx_order.try_lock() == false)
        return;

    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    // 把多线点云数组统一到一个点云中进行处理
    mtx_pointcloud.lock();
    for (int i = 0; i < N_SCANS; i++) {
        scanStartInd[i] = cloud->size() + 5;
        *cloud += laserCloudScans[i];
        scanEndInd[i] = cloud->size() - 6;
    }
    // std::cout << "cloud size: " << cloud->size() << std::endl;

    mtx_pointcloud.unlock();
    // 先在车附近取出一些点，求平面方程，然后根据筛掉高度过高的点
    // 用同一scan上的相邻点高度差作为一个特征

    //半径筛选点云
/*    
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float radius = 15.0f;
    pcl::PointXYZI searchPoint;
    searchPoint.x = 0.0f;
    searchPoint.y = 0.0f;
    searchPoint.z = 0.0f;
    kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    pcl::PointCloud<PointType>::Ptr aftRadiusSearch(new pcl::PointCloud<PointType>);
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(cloud);
    pcl::IndicesPtr indices(new std::vector<int>(pointIdxRadiusSearch));
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*aftRadiusSearch);

    pcl::copyPointCloud(*aftRadiusSearch, points_visualization);
    pub_temp_viz(points_visualization);
    std::cout << "points_visualization size: " << points_visualization.size() << std::endl;
*/
    // 多条件滤波
    pcl::PointCloud<PointType>::Ptr aftConditionalRemoval(new pcl::PointCloud<PointType>);
    pcl::ConditionAnd<PointType>::Ptr range_conditional(new pcl::ConditionAnd<PointType>());
    pcl::FieldComparison<PointType>::ConstPtr condition1(new pcl::FieldComparison<PointType> ("y", pcl::ComparisonOps::LT, 5.0));
    pcl::FieldComparison<PointType>::ConstPtr condition2(new pcl::FieldComparison<PointType> ("y", pcl::ComparisonOps::GT, -5.0));
    pcl::FieldComparison<PointType>::ConstPtr condition3(new pcl::FieldComparison<PointType> ("x", pcl::ComparisonOps::LT, 15.0));
    pcl::FieldComparison<PointType>::ConstPtr condition4(new pcl::FieldComparison<PointType> ("x", pcl::ComparisonOps::GT, -15.0));
    range_conditional->addComparison(condition1);
    range_conditional->addComparison(condition2);
    range_conditional->addComparison(condition3);
    range_conditional->addComparison(condition4);
    pcl::ConditionalRemoval<PointType> conditionalRemoval;
    conditionalRemoval.setCondition(range_conditional);
    conditionalRemoval.setInputCloud(cloud);
    conditionalRemoval.setKeepOrganized(true);
    conditionalRemoval.filter(*aftConditionalRemoval);
    // pcl::copyPointCloud<PointType>(*aftConditionalRemoval, points_visualization);
    // pub_temp_viz(points_visualization, pubTempViz2);
    // std::cout << "points_visualization size: " << points_visualization.size() << std::endl;

/*
    // ransac拟合平面--使用SACSegmentation
    pcl::ModelCoefficientsPtr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndicesPtr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointType> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.05);
    segmentation.setMaxIterations(200);
    segmentation.setProbability(0.95);
    segmentation.setInputCloud(aftConditionalRemoval);
    segmentation.segment(*inliers, *coefficients);
    std::cout << "coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << std::endl;
*/
    // RandomSampleConsensus拟合平面
    // 这个比点云分割效果稳定，分割那个会出现inlier过少无法获得coefficients的情况
    // TODO: 获取拟合后平面系数的正规操作？
    std::vector<int> inliers;
    Eigen::VectorXf plane_coefficients;
    pcl::SampleConsensusModelPlane<PointType>::Ptr model_plane(new pcl::SampleConsensusModelPlane<PointType>(aftConditionalRemoval));
    pcl::RandomSampleConsensus<PointType> ransac(model_plane);
    ransac.setDistanceThreshold(0.05);
    ransac.computeModel();
    ransac.getInliers(inliers);
    ransac.getModelCoefficients(plane_coefficients);
    // std::cout << "coefficients size: " << plane_coefficients.size() \
    //             << "    data: " << plane_coefficients.data()[0] << ' ' \
    //             << plane_coefficients.data()[1] << ' ' \
    //             << plane_coefficients.data()[2] << ' ' \
    //             << plane_coefficients.data()[3] << std::endl;
    // pcl::copyPointCloud<PointType>(*aftConditionalRemoval, inliers, points_visualization);
    // pub_temp_viz(points_visualization, pubTempViz2);
    // std::cout << "points_visualization size: " << points_visualization.size() << std::endl;

    // 提取不参与构成平面的点
    pcl::PointCloud<PointType>::Ptr outliers(new pcl::PointCloud<PointType>);
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(aftConditionalRemoval);
    // 将vector类型的index转换成pcl::Indices类型
    pcl::IndicesConstPtr indices(new std::vector<int>(inliers));
    extract.setIndices(indices); 
    // 提取不能构成平面的点
    extract.setNegative(true);
    extract.filter(*outliers);
    // 根据距离筛选可能的道路边缘点
    pcl::PointCloud<PointType>::Ptr atfDistanceSelected(new pcl::PointCloud<PointType>);
    for (auto point : outliers->points){
        float distance = distance_to_plane(point, plane_coefficients[0], plane_coefficients[1], plane_coefficients[2], plane_coefficients[3]);
        if (distance < 0.3){
            atfDistanceSelected->points.push_back(point);
        }
    }

    // pcl::copyPointCloud<PointType>(*atfDistanceSelected, points_visualization);


    // 使用NormalEstimationOMP做法线求解和曲率求解  Page:321
// /*
    pcl::PointCloud<PointType>::Ptr aftCurvature_left(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr aftCurvature_right(new pcl::PointCloud<PointType>());
    pcl::NormalEstimationOMP<PointType, pcl::PointNormal> ne;
    ne.setInputCloud(aftConditionalRemoval);
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    ne.setSearchMethod(tree);
    // 设置视点，用于唯一确定法向量具有一致的方向
    ne.setViewPoint(0, 0, 0);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>());
    // ne.setKSearch(10);
    ne.setRadiusSearch(0.5);
    ne.compute(*cloudWithNormals);
    // float max_curvature = 0;
    // float min_curvature = std::numeric_limits<float>::max();
    for(auto index = 0; index < cloudWithNormals->points.size(); ++index){
        if(cloudWithNormals->points[index].curvature > 0.0005){
            float distance = distance_to_plane(aftConditionalRemoval->points[index], plane_coefficients[0], plane_coefficients[1], plane_coefficients[2], plane_coefficients[3]);
            if (distance < 0.3 && distance > 0.02){     // 0.05时，左侧就排除在外了
                double angle = std::atan2(aftConditionalRemoval->points[index].y, aftConditionalRemoval->points[index].x);
                if(angle > 0){
                    aftCurvature_left->points.push_back(aftConditionalRemoval->points[index]);
                }
                else {
                    aftCurvature_right->points.push_back(aftConditionalRemoval->points[index]);
                }
            }
        }
        // std::cout << point.curvature << std::endl; 
        // if(point.curvature < min_curvature)
        //     min_curvature = point.curvature;
        // if(point.curvature > max_curvature)
        //     max_curvature = point.curvature;
    }
    pcl::copyPointCloud<PointType>(*aftCurvature_right, points_visualization);
    pub_temp_viz(points_visualization, pubTempViz2);
    std::cout << "points_visualization size: " << points_visualization.size() << std::endl;
    // pcl::copyPointCloud<PointType>(*aftCurvature_right, points_visualization);
    // std::cout << "max_curvature: " << max_curvature << "\tmin_curvature: " << min_curvature << std::endl;
// */

    // 使用手算的curvature，但是这个提取出来的点的效果甚至不如直接按点到平面距离提取的那个好
/*
    // float max_curvature = 0;
    // float min_curvature = std::numeric_limits<float>::max();
    float cloudCurvature[400000] = {0};
    pcl::PointCloud<PointType>::Ptr aftCurvature(new pcl::PointCloud<PointType>());
    cloud = aftConditionalRemoval;
    for (int i = 5; i < cloud->size() - 5; ++i){
        float diffX = cloud->points[i - 5].x + cloud->points[i - 4].x + cloud->points[i - 3].x + cloud->points[i - 2].x + cloud->points[i - 1].x - 10 * cloud->points[i].x + cloud->points[i + 1].x + cloud->points[i + 2].x + cloud->points[i + 3].x + cloud->points[i + 4].x + cloud->points[i + 5].x;
        float diffY = cloud->points[i - 5].y + cloud->points[i - 4].y + cloud->points[i - 3].y + cloud->points[i - 2].y + cloud->points[i - 1].y - 10 * cloud->points[i].y + cloud->points[i + 1].y + cloud->points[i + 2].y + cloud->points[i + 3].y + cloud->points[i + 4].y + cloud->points[i + 5].y;
        float diffZ = cloud->points[i - 5].z + cloud->points[i - 4].z + cloud->points[i - 3].z + cloud->points[i - 2].z + cloud->points[i - 1].z - 10 * cloud->points[i].z + cloud->points[i + 1].z + cloud->points[i + 2].z + cloud->points[i + 3].z + cloud->points[i + 4].z + cloud->points[i + 5].z;

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        if(cloudCurvature[i] > 5)
            aftCurvature->points.push_back(cloud->points[i]);
        // std::cout << cloudCurvature[i] << std::endl;
        // if(cloudCurvature[i] < min_curvature)
        //     min_curvature = cloudCurvature[i];
        // if(cloudCurvature[i] > max_curvature)
        //     max_curvature = cloudCurvature[i];
    
    }
    pcl::copyPointCloud<PointType>(*aftCurvature, points_visualization);
    // std::cout << "max_curvature: " << max_curvature << "\tmin_curvature: " << min_curvature << std::endl;
*/
    // 使用pcl内置的no_nurbs进行样条曲线拟合（因为默认是闭合曲线，所以拟合不成功）
/*
    pcl::ModelCoefficientsPtr coefficients(new pcl::ModelCoefficients);
    coefficients->values.push_back(plane_coefficients[0]);
    coefficients->values.push_back(plane_coefficients[1]);
    coefficients->values.push_back(plane_coefficients[2]);
    coefficients->values.push_back(plane_coefficients[3]);
    pcl::PointCloud<PointType>::Ptr aftfitting = fitting_curve_to_2d_spline(aftCurvature_left, coefficients);
*/
    pcl::PointCloud<PointType>::Ptr aftfitting(new pcl::PointCloud<PointType>);
    Eigen::VectorXf curve_coefficients = FitterLeastSquareMethod(aftCurvature_right);
    float X_min, X_max ;
    getMaxAndMin(aftCurvature_right, X_min, X_max);
    restorePointsFrom2D(aftfitting, X_min, X_max, curve_coefficients, plane_coefficients);
    if (aftfitting != nullptr) {
        pcl::copyPointCloud<PointType>(*aftfitting, points_visualization);
        pub_temp_viz(points_visualization);
        std::cout << "points_visualization size: " << points_visualization.size() << std::endl;
    }


    for (int i = 0; i < N_SCANS;  i++){
        laserCloudScans[i].clear();
    }
}

// 去除无效点云
// 投影二维化
void laserCloudHandler(sensor_msgs::PointCloud2::ConstPtr cloud){
    pcl::fromROSMsg(*cloud, source_points); 
    msgHeader = cloud->header;

    std::vector<int> index;
    pcl::removeNaNFromPointCloud(source_points, source_points, index);

    int cloud_size = source_points.points.size();
    // ROS_WARN("cloud_size: %d", cloud_size);

    float startOrientation = atan2(source_points.points[0].y, source_points.points[0].x);
    float endOrientation = atan2(source_points.points[cloud_size - 1].y, 
                                source_points.points[cloud_size - 1].x) + 
                            2 * M_PI;
    float deltaOrientation = endOrientation - startOrientation;

    // 认为startOrientation与endOrientation不会相差到π这么大的情况
    if (endOrientation - startOrientation > 3 * M_PI) {
        deltaOrientation = endOrientation - startOrientation - 2 * M_PI;
    }
    if (endOrientation - startOrientation < M_PI) {
        deltaOrientation = endOrientation - startOrientation + 2 * M_PI;
    }
    PointType point;
    for (int i = 0; i < cloud_size;  i++) {
        // 水平放置的话，高度大于0的点不认为是地面点（虽然有上坡情况）
        if (source_points.points[i].z > 0)
            continue;

        point.x = source_points.points[i].x;
        point.y = source_points.points[i].y;
        point.z = source_points.points[i].z;

        float angleZ_XY = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        if(N_SCANS == 64){
            // 最下面的scanID是0
            scanID = int((angleZ_XY - bottom_angle) / angle_resolution + 0.5);
            if(scanID < 0 || scanID >= N_SCANS)
                continue;

            point.intensity = scanID;
            laserCloudScans[scanID].push_back(point);
            float angleY_XZ = atan2(point.y, point.x) * 180 / M_PI;
            // 车辆前部中心线高度
            if(fabs(angleY_XZ) < 5)
                centralHeights[2 * scanID].push_back(point.z);
            else if(fabs(fabs(angleY_XZ) - 180) < 5)
                centralHeights[2 * scanID + 1].push_back(point.z);

        }
    }

    // 求个平均
    for(int i = 0; i < 2 * N_SCANS; i++){
        float tempHeight = 0;
        float num = centralHeights[i].size();
        if(num == 0) continue;
        for(auto data : centralHeights[i])
            tempHeight += data;
        centralHeights[i].clear();
        centralHeights[i].push_back(tempHeight / num);
    }
    mtx_order.unlock();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "roadEdge_Extract");
    ros::NodeHandle nh;

    std::string topic_name;
    // nh.param<std::string>("topic_name", topic_name, "/velodyne_points");
    nh.param<std::string>("topic_name", topic_name, "/laser_cloud_manual");
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(topic_name, 100, laserCloudHandler);

    pubRoadEdge = nh.advertise<sensor_msgs::PointCloud2>("/roadEdge", 100);
    pubTempViz = nh.advertise<sensor_msgs::PointCloud2>("/TempViz", 100);
    pubTempViz2 = nh.advertise<sensor_msgs::PointCloud2>("/TempViz2", 100);

    mtx_order.lock();
    ROS_INFO_STREAM("ready to extract roadEdge...");
    ros::Rate rate(50);
    while (ros::ok()) {
        roadEdgeExtract();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}


// pcl官方教程中例子摘要， 是先拟合surface，再进一步拟合curve
void fitting_curve_spline_in_pcl_tutorials(pcl::PointCloud<PointType>::Ptr cloud){
    // ############################################################################
    // fit B-spline surface
    pcl::on_nurbs::NurbsDataSurface data;
    PointCloud2Vector3d(cloud, data.interior);

    // parameters
    unsigned order(3);
    unsigned refinement(5);
    unsigned iterations(10);
    unsigned mesh_resolution(256);

    pcl::on_nurbs::FittingSurface::Parameter params;
    params.interior_smoothness = 0.2;
    params.interior_weight = 1.0;
    params.boundary_smoothness = 0.2;
    params.boundary_weight = 0.0;

    // initialize
    ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &data);
    pcl::on_nurbs::FittingSurface fit(&data, nurbs);

    // surface refinement
    for (unsigned i = 0; i < refinement; i++)
    {
        fit.refine (0);
        fit.refine (1);
        fit.assemble (params);
        fit.solve ();
    }
      // surface fitting with final refinement level
    for (unsigned i = 0; i < iterations; i++)
    {
        fit.assemble (params);
        fit.solve ();
    }

    // ############################################################################
    // fit B-spline curve

    // parameters
    pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
    curve_params.addCPsAccuracy = 5e-2;
    curve_params.addCPsIteration = 3;
    curve_params.maxCPs = 200;
    curve_params.accuracy = 1e-3;
    curve_params.iterations = 100;

    curve_params.param.closest_point_resolution = 0;
    curve_params.param.closest_point_weight = 1.0;
    curve_params.param.closest_point_sigma2 = 0.1;
    curve_params.param.interior_sigma2 = 0.00001;
    curve_params.param.smooth_concavity = 1.0;
    curve_params.param.smoothness = 1.0;

    // initialisation (circular)
    pcl::on_nurbs::NurbsDataCurve2d curve_data;
    curve_data.interior = data.interior_param;
    curve_data.interior_weight_function.push_back (true);
    ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D (order, curve_data.interior);

    // curve fitting
    pcl::on_nurbs::FittingCurve2dASDM curve_fit (&curve_data, curve_nurbs);
    // curve_fit.setQuiet (false); // enable/disable debug output
    curve_fit.fitting (curve_params);
}

void getMaxAndMin(pcl::PointCloud<PointType>::Ptr cloud, float &X_min, float &X_max){
    X_min = 100000.0f;
    X_max = -100000.0f;
    for(auto point : cloud->points){
        if (X_max < point.x) 
            X_max = point.x;
        if (X_min > point.x)
            X_min = point.x;
    }
}

// 恢复曲线到3D坐标，可以在rviz中进行展示，需要针对起止x坐标进行采样
// 给定x坐标值，通过拟合的曲线方程获得y坐标值，再通过拟合的平面方程获得z坐标值
// @param resolution: 采样点总数
void restorePointsFrom2D(pcl::PointCloud<PointType>::Ptr cloud, float X_min, float X_max, const Eigen::VectorXf& coefficients_curve, const Eigen::VectorXf &coefficients_plane, int resolution){
    cloud->clear();
    std::vector<float> X;

    float delta = (X_max - X_min) / resolution;
    float x_c = X_min;
    while (x_c < X_max) {
        X.push_back(x_c);
        x_c += delta;    
    }
    try {
        for (int i = 0; i < X.size(); ++i) {
            pcl::PointXYZI temp_point;
            temp_point.x = X[i];
            // TODO: 这里阶数不一定是3阶，因此需要修改通用的算法
            temp_point.y = coefficients_curve[0] +\
                            coefficients_curve[1] * temp_point.x +\
                            coefficients_curve[2] * temp_point.x * temp_point.x + \
                            coefficients_curve[3] * temp_point.x * temp_point.x * temp_point.x;
            temp_point.z =  -(coefficients_plane[0] * temp_point.x + coefficients_plane[1] * temp_point.y + coefficients_plane[3]) / coefficients_plane[2];
            temp_point.intensity = std::rand();
            cloud->points.push_back(temp_point);
        }
        std::cout << "restorePointsFrom2D finished!" << std::endl;
    } catch (std::exception E) {
        std::cout << E.what() << std::endl;
    }
}
/**
 * @brief Fit polynomial using Least Square Method.
 * @param cloud_in point cloud data input
 * @param cloud_out point cloud data output after fitting
 * @param orders Fitting order which should be larger than zero. 
 * @return Eigen::VectorXf Coefficients vector of fitted polynomial.
 */
Eigen::VectorXf FitterLeastSquareMethod(pcl::PointCloud<PointType>::Ptr cloud_in, uint8_t orders){

    std::vector<float> X;
    std::vector<float> Y;
    PointCloud2Vector2d(cloud_in, X, Y);
    // abnormal input verification
    if (X.size() < 2 || Y.size() < 2 || X.size() != Y.size() || orders < 1){
        ROS_ERROR("too less points, please check!");
        exit(EXIT_FAILURE);
    }

    // map sample data from STL vector to eigen vector
    Eigen::Map<Eigen::VectorXf> sampleX(X.data(), X.size());
    Eigen::Map<Eigen::VectorXf> sampleY(Y.data(), Y.size());

    Eigen::MatrixXf mtxVandermonde(X.size(), orders + 1);  // Vandermonde matrix of X-axis coordinate vector of sample data
    Eigen::VectorXf colVandermonde = sampleX;              // Vandermonde column

    // construct Vandermonde matrix column by column
    for (size_t i = 0; i < orders + 1; ++i)
    {
        if (0 == i)
        {
            mtxVandermonde.col(0) = Eigen::VectorXf::Constant(X.size(), 1, 1);
            continue;
        }
        if (1 == i)
        {
            mtxVandermonde.col(1) = colVandermonde;
            continue;
        }
        colVandermonde = colVandermonde.array()*sampleX.array();
        mtxVandermonde.col(i) = colVandermonde;
    }

    // calculate coefficients vector of fitted polynomial
    // 元素按阶数从低到高排列
    Eigen::VectorXf result = (mtxVandermonde.transpose()*mtxVandermonde).inverse()*(mtxVandermonde.transpose())*sampleY;

    return result;
}