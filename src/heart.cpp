//
// Created by Rvosuke on 2024/9/24.
//

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <cmath>
#include <cstdlib>
#include <random>

auto clamp = [](float val, float min, float max) {
    return (val < min) ? min : (val > max) ? max : val;
};

pcl::PointCloud<pcl::PointXYZRGB>::Ptr generateSmoothHeartPointCloud(int num_points, float scale) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> distrib_u(0, 1.0); // u的正态分布，均值在M_PI/2附近
    std::normal_distribution<> distrib_v(0, 1.0);     // v的正态分布，均值在M_PI附近
    std::normal_distribution<> distrib_s(0.5, 1.0);

    for (int i = 0; i < num_points; ++i) {
//        float u = clamp(distrib_u(gen), -M_PI, M_PI); // 限制u在[0, M_PI]之间
//        float v = clamp(distrib_v(gen), -M_PI * 2, M_PI * 2); // 限制v在[0, 2 * M_PI]之间
        float u = distrib_u(gen);
        float v = distrib_v(gen);
        scale = clamp(distrib_s(gen)*0.1, 0, 1); // 缩放因子


        // 使用心形方程生成三维点
        float f = scale * 16 * pow(sin(u), 3);
        float g = scale * (13 * cos(u) - 5 * cos(2 * u) - 2 * cos(3 * u) - cos(4 * u));

        float x = f * cos(v);
        float y = g;
        float z = f * sin(v) / 2;
        pcl::PointXYZRGB point;
        point.x = x;
        point.y = y;
        point.z = z;

        // 根据距离实现粉红色渐变
//        float distance = sqrt(x * x + y * y + z * z); // 到中心的距离
//        float normalized_distance = distance / (scale * 24); // 距离归一化

        point.r = static_cast<uint8_t>(255 * scale * 2); // 红色通道
        point.g = static_cast<uint8_t>(182 * scale * 2); // 绿色通道
        point.b = static_cast<uint8_t>(193 * scale * 2); // 蓝色通道
//        point.r = 255;
//        point.g = 182;
//        point.b = 193;

        cloud->points.push_back(point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    return cloud;
}


void animateHeart(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::visualization::CloudViewer& viewer) {
    for (size_t i = 1; i <= cloud->points.size(); i += 3e5) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr partial_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        partial_cloud->points.insert(partial_cloud->points.end(), cloud->points.begin(), cloud->points.begin() + i);
        viewer.showCloud(partial_cloud);
        pcl_sleep(0.05); // 控制每帧的速度
    }
}

int heart() {
    int num_points = 3e6;
    float scale = 1;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr heart_cloud = generateSmoothHeartPointCloud(num_points, scale);
    pcl::visualization::CloudViewer viewer("Animated 3D Heart");
//    viewer.showCloud(heart_cloud);


    while (!viewer.wasStopped()) {
        // 保持窗口打开
        animateHeart(heart_cloud, viewer);
    }

    // 保存为PCD文件
    pcl::io::savePCDFileASCII("heart.pcd", *heart_cloud);

    return 0;
}
