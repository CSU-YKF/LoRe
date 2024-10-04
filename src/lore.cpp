//
// Created by Rvosuke on 2024/9/8.
//
#include <pcl/registration/icp.h>
#include "../include/lore.h"


std::vector<float> computePointDistances(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    float min_dist, max_dist, avg_dist = 0;
    // 初始化变量
    min_dist = std::numeric_limits<float>::max();
    max_dist = 0.0;
    float total_dist = 0.0;
    int point_count = 0;

    // 使用KD-Tree加速最近邻搜索
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    // 遍历点云中的每个点
    for (auto point: cloud->points) {
        std::vector<int> pointIdxNKNSearch(2); // 搜索最近的两个点（包括自身）
        std::vector<float> pointNKNSquaredDistance(2);

        if (tree.nearestKSearch(point, 2, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            float dist = std::sqrt(pointNKNSquaredDistance[1]); // 最近邻距离

            // 更新最小值、最大值
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;

            total_dist += dist;
            point_count++;
        }
    }

    // 计算平均距离
    if (point_count > 0) {
        avg_dist = total_dist / float(point_count);
    }

    return {min_dist, max_dist, avg_dist};
}

std::vector<float> geometryFitting(PointCloud &cloud_in, float param_distance, pcl::ModelCoefficients::Ptr &coefficients) {
    // 圆拟合
    pcl::PointIndices::Ptr inlines(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(param_distance);
    seg.setMaxIterations(100);
    seg.setInputCloud(cloud_in.makeShared());
    seg.segment(*inlines, *coefficients);

    Eigen::Vector3f center(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    Eigen::Vector3f normal(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
    float radius = coefficients->values[6];

    return {center.x(), center.y(), center.z(), normal.x(), normal.y(), normal.z(), radius};
}

void view_cloud(PointCloud &cloud, const std::string &window_name = "Cloud Viewer") {
    pcl::visualization::CloudViewer viewer(window_name);
    viewer.showCloud(cloud.makeShared());

    while (!viewer.wasStopped()) {}
}

// 创建一个用于可视化的PCL Viewer，并显示原始点云和拟合的圆
void view_cloud(PointCloud &cloud, const std::vector<float>& circle_params) {
    pcl::visualization::CloudViewer viewer("cloud");

    // 创建一个带RGB信息的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& point : cloud) {
        pcl::PointXYZRGB p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        p.r = 255;
        p.g = 255;
        p.b = 255;  // 设置原始点云为白色
        cloud_rgb->push_back(p);
    }

    // 生成拟合的圆形并添加红色
    Eigen::Vector3f center(circle_params[0], circle_params[1], circle_params[2]);
    Eigen::Vector3f normal(circle_params[3], circle_params[4], circle_params[5]);
    float radius = circle_params[6];

    // 创建圆形点
    const int num_circle_points = 100; // 圆的点数
    for (int i = 0; i < num_circle_points; ++i) {
        float theta = 2.0f * M_PI * static_cast<float>(i) / static_cast<float>(num_circle_points);
        Eigen::Vector3f point_on_circle = center + radius * Eigen::Vector3f(cos(theta), sin(theta), 0.0f);
        pcl::PointXYZRGB p;
        p.x = point_on_circle.x();
        p.y = point_on_circle.y();
        p.z = point_on_circle.z();
        p.r = 255;
        p.g = 0;
        p.b = 0;
        cloud_rgb->push_back(p);
    }

    viewer.showCloud(cloud_rgb);
    while (!viewer.wasStopped()) {}
}


// 使用ICP配准进行拟合判断
bool icp_registration(const PointCloud::Ptr& source_cloud, const PointCloud::Ptr& target_cloud) {
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);

    PointCloud::Ptr final_cloud(new PointCloud);
    icp.align(*final_cloud);

    std::cout << "ICP has converged: " << icp.hasConverged() << " with score: " << icp.getFitnessScore() << std::endl;

    return icp.hasConverged() && icp.getFitnessScore() < 50; // 设置一个阈值来判断配准的拟合效果
}

int lore(int argc, char **argv) {
    PointCloud::Ptr cloud_in(new PointCloud);
    pcl::io::loadPCDFile(R"(../datasets/experiment_box/3926originPointCloud.pcd)", *cloud_in);
    // 读取生成的3D圆形PCD文件
    PointCloud::Ptr circle_cloud(new PointCloud);
    pcl::io::loadPCDFile("../datasets/circle_3d.pcd", *circle_cloud);

    std::vector<float> box_min = {-450, 0, 1000};
    std::vector<float> box_max = {750, 200, 1300};

    std::vector<int> indices;
    printt("Cloud before filtering: ", cloud_in->size());
    cropBox(*cloud_in, box_min, box_max);
    printt("Cloud after CropBox: ", cloud_in->size());

    std::vector<float> distances = computePointDistances(cloud_in);
    float avg_dist = distances[2] * 3;
    printt("Average distance: ", avg_dist);
    removeOutlier(*cloud_in, avg_dist);
    printt("Cloud after outlier removal: ", cloud_in->size());
//    view_cloud(*cloud_in);

    downSampling(*cloud_in, avg_dist);

    std::vector<PointCloud::Ptr> cloud_interest = segmentation(*cloud_in, avg_dist, cloud_in->size());
    for (const auto &cloud: cloud_interest) {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        view_cloud(*cloud);
        if (icp_registration(cloud, circle_cloud)) {
            std::vector<float> result = geometryFitting(*cloud, avg_dist, coefficients);
            printt("Circle center: ", result[0], " ", result[1], " ", result[2]);
            printt("Circle normal: ", result[3], " ", result[4], " ", result[5]);
//            view_cloud(*cloud, result);
        }
    }
    return 0;
}
