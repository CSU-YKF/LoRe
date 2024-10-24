//
// Created by Rvosuke on 2024/10/7.
//

#include "../include/fit.h"


std::vector<float> geometryFitting(PointCloud &cloud_in, float param_distance) {
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

    Eigen::Vector3f center(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    Eigen::Vector3f normal(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
    float radius = coefficients->values[6];

    return {center.x(), center.y(), center.z(), normal.x(), normal.y(), normal.z(), radius};
}

bool icp_registration(const PointCloud::Ptr& source_cloud, const PointCloud::Ptr& target_cloud, float threshold) {
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);

    PointCloud::Ptr final_cloud(new PointCloud);
    icp.align(*final_cloud);

    printt("ICP has converged:", icp.hasConverged(), " with score: ", icp.getFitnessScore());
    return icp.hasConverged() && icp.getFitnessScore() < threshold; // Set a threshold to determine how well the registration fits
}
