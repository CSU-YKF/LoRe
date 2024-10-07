//
// Created by Rvosuke on 2024/9/9.
//

#include "../include/segmentation.h"


void planarSegmentation(PointCloud &cloud_in, const float param_distance) {
    PointCloud cloud_filtered = cloud_in;
    PointCloud cloud_instance;
    PointCloud cloud_sum;

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(param_distance);
    seg.setMaxIterations(1000);

    // Set the stop condition to the point cloud less than 30% of the original point cloud to avoid detecting irrelevant noise, which can speed up the processing
    for (int i = 0; i < 100 && cloud_filtered.size() > 0.3 * cloud_in.size(); i++) {
        pcl::PointIndices::Ptr inlines(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::ExtractIndices<PointT> extract;

        seg.setInputCloud(cloud_filtered.makeShared());
        seg.segment(*inlines, *coefficients);

        if (inlines->indices.empty()) continue;

        extract.setInputCloud(cloud_filtered.makeShared());
        extract.setIndices(inlines);

        // Check that the plane is perpendicular to the z-axis
        // Set the size of the indices to not be too small, so that it can play a role in denoising
        if (std::abs(coefficients->values[2]) > 0.9 && inlines->indices.size() > 0.1 * cloud_in.size()) {
            extract.setNegative(false);
            extract.filter(cloud_instance);
            cloud_sum += cloud_instance;
        }

        // Removes the split planar point from the cloud_filtered
        extract.setNegative(true);  // extract the remaining point clouds
        extract.filter(cloud_filtered);
    }
    cloud_in = cloud_sum;
}


std::vector<PointCloud::Ptr> cloudClustering(PointCloud &cloud_in, float param_distance) {
    PointCloud::Ptr cloud = cloud_in.makeShared();
    int point_size = cloud->size();

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    tree->setInputCloud(cloud);
    ec.setClusterTolerance(param_distance);
    // The min cluster size is important, and if it is set too small, some noise points will be mistaken for clusters
    ec.setMinClusterSize(point_size / 20);
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
//        std::string filename = "cluster_" + std::to_string(cluster_id) + ".pcd";
//        pcl::io::savePCDFileASCII(filename, *cloud_cluster);
        clusters.push_back(cloud_cluster);
    }

    return clusters;
}


