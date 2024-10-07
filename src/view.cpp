//
// Created by Rvosuke on 2024/9/19.
//

#include "../include/view.h"


std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
    return ss.str();
}

void view_cloud(PointCloud &cloud, const std::string &window_name) {
    pcl::visualization::CloudViewer viewer(window_name);
    viewer.showCloud(cloud.makeShared());

    while (!viewer.wasStopped()) {}
}


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