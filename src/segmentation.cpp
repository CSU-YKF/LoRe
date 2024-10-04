//
// Created by Rvosuke on 2024/9/9.
//
#include "../include/segmentation.h"


std::vector<PointCloud::Ptr> segmentation(PointCloud &cloud_in, float param_distance, int point_size) {
    PointCloud::Ptr cloud = cloud_in.makeShared();
    // 点云聚类
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(param_distance);
    // min cluster size 比较重要，如果设置太小，会导致一些噪声点被误认为是簇；如果设置太大，会导致无法检测到目标。
    ec.setMinClusterSize(point_size / 10);
    ec.setMaxClusterSize(point_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    printt("Number of clusters: ", cluster_indices.size());

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
        printt("Cluster ", cluster_id, " size: ", cloud_cluster->size());
        cluster_id++;
        clusters.push_back(cloud_cluster);
    }

    return clusters;
}
