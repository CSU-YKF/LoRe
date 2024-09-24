//
// Created by Rvosuke on 2024/9/8.
//
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

std::vector<float> geometryFitting(PointCloud &cloud_in, float param_distance) {
    // 圆拟合
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inlines(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(param_distance);
    seg.setMaxIterations(100);
    seg.setInputCloud(cloud_in.makeShared());
    seg.segment(*inlines, *coefficients);

    int inside_circle_count = 0;
    // 提取圆的参数：圆心 (coefficients->values[0], coefficients->values[1], coefficients->values[2])
    // 半径 coefficients->values[3]
    double cx = coefficients->values[0];
    double cy = coefficients->values[1];
    double cz = coefficients->values[2];
    double radius = (coefficients->values[3] + coefficients->values[4] + coefficients->values[5]) / 3;

    // 遍历点云，计算每个点到圆心的距离
    for (const auto &point: cloud_in.points) {
        double distance_to_center = std::sqrt((point.x - cx) * (point.x - cx) +
                                              (point.y - cy) * (point.y - cy));

        // 判断该点是否在圆内部
        if (distance_to_center <= radius) {
            inside_circle_count++;
        }
    }

    // 计算圆内部点的比例
    double inside_ratio = static_cast<double>(inside_circle_count) / cloud_in.size();


    if (inside_ratio > 0.9) {
        // 法向量估计
        Eigen::Vector3f center(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        Eigen::Vector3f normal(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
        std::cout << "Circle center: " << center << std::endl;
        std::cout << "Circle normal: " << normal << std::endl;
        return {center.x(), center.y(), center.z(), normal.x(), normal.y(), normal.z()};
    }

    std::cerr << "Circle fitting failed!" << std::endl;
    return {};
}


int lore(int argc, char **argv) {
    PointCloud::Ptr cloud_in(new PointCloud);
    PointCloud::Ptr cloud_load(new PointCloud);
    PointCloud::Ptr cloud_recycle(new PointCloud);
    pcl::io::loadPCDFile(R"(D:\LoRe\datasets\source.pcd)", *cloud_in);
    pcl::io::loadPCDFile(R"(D:\LoRe\datasets\sor_pass_ly.pcd)", *cloud_load);
    pcl::io::loadPCDFile(R"(D:\LoRe\datasets\sor_pass_ry.pcd)", *cloud_recycle);

    std::vector<int> indices;
    std::cout << "Cloud before filtering: " << cloud_in->size() << std::endl;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
    pcl::removeNaNFromPointCloud(*cloud_load, *cloud_load, indices);
    pcl::removeNaNFromPointCloud(*cloud_recycle, *cloud_recycle, indices);
    std::cout << "Cloud after removing NaN: " << cloud_in->size() << std::endl;

    downSampling(*cloud_in);
    std::vector<float> distances = computePointDistances(cloud_in);
    float avg_dist = distances[2];
    printt("Average distance after down sampling: ", avg_dist);
    removeOutlier(*cloud_in, avg_dist);


    std::vector<PointCloud::Ptr> cloud_interest = segmentation(*cloud_in, avg_dist, cloud_in->size());
    for (const auto &cloud: cloud_interest) {
        pcl::visualization::CloudViewer viewer3("Cloud Viewer");
        viewer3.showCloud(cloud);
        while (!viewer3.wasStopped()) {}
        // 进行几何拟合
        std::vector<float> result = geometryFitting(*cloud, avg_dist);
        std::cout << result[3] << result[4] << result[5] << std::endl;
    }

    return 0;
}
