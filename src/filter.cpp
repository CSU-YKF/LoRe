//
// Created by Rvosuke on 2024/9/8.
//
#include "filter.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


void downSampling(PointCloud &cloud_in, const float param_distance) {
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud_in.makeShared());
    vg.setLeafSize(3 * param_distance, 3 * param_distance, 3 * param_distance);
    vg.filter(cloud_in);
    printt("Cloud after Voxel Grid: ", cloud_in.size());
}


void removeOutlier(PointCloud &cloud_in, const float param_distance) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inlines(new pcl::PointIndices);

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(3 * param_distance);
    seg.setMaxIterations(1000);

    int nr_points = (int) cloud_in.size();
    while (cloud_in.size() > int(0.3 * nr_points) && cloud_in.size() > int(1e4)) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_in.makeShared());
        seg.segment(*inlines, *coefficients);

        // Extract the planar inlines from the input cloud
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_in.makeShared());
        extract.setIndices(inlines);
        extract.setNegative(true);
        extract.filter(cloud_in);
        printt("After remove planar component: ", cloud_in.size());
    }
}


static int statisticalRemoval(PointCloud &cloud_in) {
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_in.makeShared());
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(cloud_in);
    return 0;
}