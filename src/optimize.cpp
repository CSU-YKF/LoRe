//
// Created by Rvosuke on 2024/10/7.
//

#include "../include/optimize.h"


std::vector<float> computePointDistances(const PointCloud::Ptr &cloud) {
    float min_dist, max_dist, avg_dist = 0;
    min_dist = std::numeric_limits<float>::max();
    max_dist = 0.0;
    float total_dist = 0.0;
    int point_count = 0;

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    for (auto point: cloud->points) {
        std::vector<int> pointIdxNKNSearch(2); // Search for the two nearest points (including itself)
        std::vector<float> pointNKNSquaredDistance(2);

        if (tree.nearestKSearch(point, 2, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            float dist = std::sqrt(pointNKNSquaredDistance[1]); // nearest neighbor distance

            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;

            total_dist += dist;
            point_count++;
        }
    }

    if (point_count > 0) {
        avg_dist = total_dist / float(point_count);
    }

    return {min_dist, max_dist, avg_dist};
}

void normalEstimation(PointCloud &cloud_in, pcl::PointCloud<pcl::Normal>::Ptr &normals) {
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud_in.makeShared());
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);
    ne.setKSearch(26);
    ne.compute(*normals);
}
