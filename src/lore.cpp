//
// Created by Rvosuke on 2024/9/8.
//
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/visualization/cloud_viewer.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


static PointCloud filte(PointCloud &cloud_in) {
    std::vector<int> indices;
    PointCloud::Ptr cloud_filtered(new PointCloud);
    std::cout << "Cloud before filtering: " << cloud_in.size() << std::endl;
    pcl::removeNaNFromPointCloud(cloud_in, cloud_in, indices);
    std::cout << "Cloud after removing NaN: " << cloud_in.size() << std::endl;

    // 使用统计滤波（Statistical Outlier Removal）或体素网格下采样（Voxel Grid Down sampling）来降低点云的噪声和密度，优化处理速度。
    // 由于Statistical Outlier Removal算法的速度太慢，因此使用Voxel Grid Down sampling算法。
//    pcl::StatisticalOutlierRemoval<PointT> sor;
//    sor.setInputCloud(cloud_in.makeShared());
//    sor.setMeanK(50);
//    sor.setStddevMulThresh(1.0);
//    sor.filter(cloud_filtered);
//    std::cout << "Cloud after Statistical Outlier Removal: " << cloud_filtered.size() << std::endl;

    pcl::VoxelGrid<PointT> vg;
    PointCloud::Ptr cloud_vg(new PointCloud);
    vg.setInputCloud(cloud_in.makeShared());
    vg.setLeafSize(3, 3, 3);
    vg.filter(*cloud_vg);
    std::cout << "Cloud after Voxel Grid: " << cloud_vg->size() << std::endl;
    *cloud_filtered = *cloud_vg;

    // 使用平面分割算法（如RANSAC）将地面或不相关点剔除，仅保留与加注口和回收口相关的点。
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    PointCloud::Ptr cloud_plane(new PointCloud), cloud_f(new PointCloud);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(2);  // 0.01m
    seg.setMaxIterations(1000);

    int nr_points = (int) cloud_filtered->size();
    while (cloud_filtered->size() > 0.3 * nr_points) {
        std::cout << "Cloud before RANSAC: " << cloud_filtered->size() << std::endl;
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points."
                  << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    std::cout << "Cloud after RANSAC: " << cloud_filtered->size() << std::endl;

    return *cloud_filtered;
}

/**
 * @brief 将加注口和回收口从点云数据中分离出来，以便进一步拟合其位姿。
 * 步骤:
 * 点云聚类：使用Euclidean Cluster Extraction算法对点云进行聚类，将罐车的不同结构（如加注口、回收口、罐体等）分离成不同的点云簇。
 * 感兴趣区域提取：根据罐车的结构和已知几何布局，对位于加注口和回收口区域的点云进行提取。
 * 此时可结合RGB图像进行辅助，如果图像质量较好，可以使用颜色信息增强分割精度。
 *
 * @param cloud_in 输入点云
 */
static std::vector<PointCloud::Ptr> segmentation(PointCloud &cloud_in) {
    PointCloud::Ptr cloud = cloud_in.makeShared();
    // 点云聚类
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(3);
    ec.setMinClusterSize(1000);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    std::cout << "Number of clusters: " << cluster_indices.size() << std::endl;

    // 创建一个PointCloud数组，每个元素代表一个点云簇
    std::vector<PointCloud::Ptr> clusters;
    int cluster_id = 0;
    for (const auto &cluster: cluster_indices) {
        PointCloud::Ptr cloud_cluster(new PointCloud);
        for (const auto &idx: cluster.indices) {
            cloud_cluster->push_back((*cloud)[idx]);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        std::cout << "Cluster " << cluster_id << " size: " << cloud_cluster->size() << std::endl;
        cluster_id++;
        clusters.push_back(cloud_cluster);
    }

    return clusters;
}


/**
 * @brief 对加注口和回收口的点云数据进行几何拟合(拟合圆形结构)，得到其位姿。
 * 步骤:
 * 圆拟合：使用PCL中的RANSAC或Least Squares Circle Fitting（最小二乘圆拟合）算法，基于点云提取加注口和回收口的圆形边缘，拟合出圆心位置和法向量。
 * RANSAC适用于含有噪声的数据，它能通过迭代地随机选择点集进行拟合，并找到最优解。
 * 法向量估计：通过拟合圆的点集计算其法向量。如果圆面不平整或有偏差，可以结合主成分分析（PCA）法确定法向量方向。
 *
 * @param cloud_in 输入点云
 * @return 输出为圆心的3D坐标 $(x, y, z)$ 和法向量 $(n_x, n_y, n_z)$。
 */

static std::vector<float> geometryFitting(PointCloud &cloud_in) {
    // 圆拟合
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(1);
    seg.setMaxIterations(100);
    seg.setInputCloud(cloud_in.makeShared());
    seg.segment(*inliers, *coefficients);

    // 法向量估计
    Eigen::Vector3f center(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    Eigen::Vector3f normal(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
    std::cout << "Circle center: " << center << std::endl;
    std::cout << "Circle normal: " << normal << std::endl;

    return {center.x(), center.y(), center.z(), normal.x(), normal.y(), normal.z()};
}


int main() {
    PointCloud::Ptr cloud_in(new PointCloud);
    PointCloud::Ptr cloud_load(new PointCloud);
    PointCloud::Ptr cloud_recycle(new PointCloud);
    pcl::io::loadPCDFile(R"(D:\LoRe\datasets\source.pcd)", *cloud_in);
    pcl::io::loadPCDFile(R"(D:\LoRe\datasets\sor_pass_ly.pcd)", *cloud_load);
    pcl::io::loadPCDFile(R"(D:\LoRe\datasets\sor_pass_ry.pcd)", *cloud_recycle);

    PointCloud cloud_in_filtered = filte(*cloud_in);
//    pcl::visualization::CloudViewer viewer("Cloud Viewer");
//    viewer.showCloud(cloud_in_filtered.makeShared());
//    while (!viewer.wasStopped()) {}

    std::vector<PointCloud::Ptr> cloud_interest = segmentation(cloud_in_filtered);
    for (const auto &cloud: cloud_interest) {
//        pcl::visualization::CloudViewer viewer2("Cloud Viewer");
//        viewer2.showCloud(cloud);
        // 进行几何拟合
        std::vector<float> result = geometryFitting(*cloud);
        std::cout << "Center: (" << result[0] << ", " << result[1] << ", " << result[2] << ")" << std::endl;
//        while (!viewer2.wasStopped()) {}
    }

    return 0;
}