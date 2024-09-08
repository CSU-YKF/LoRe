//
// Created by Rvosuke on 2024/9/8.
//
#include "iostream"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/visualization/cloud_viewer.h"

int passThrough() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile(R"(D:\LoRe\datasets\example.pcd)", cloud_blob);
    pcl::fromPCLPointCloud2(cloud_blob, *cloud);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 5000.0);
    pass.filter(*cloud_filtered);

    pcl::visualization::CloudViewer viewer("Pass Through");
    viewer.showCloud(cloud_filtered);
    while (!viewer.wasStopped()) {}

    return 0;
}