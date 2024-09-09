//
// Created by Rvosuke on 2024/9/8.
//
#include "iostream"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/filter.h"

int statisticalRemoval() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(R"(D:\LoRe\datasets\example.pcd)", *cloud);
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << cloud->size() << std::endl;

//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    sor.setInputCloud(cloud);
//    sor.setMeanK(50);
//    sor.setStddevMulThresh(1.0);
//    sor.filter(*cloud_filtered);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud_filtered, indices);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << cloud_filtered->size() << std::endl;
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud_filtered);

    while (!viewer.wasStopped()) {}

    return 0;
}