//
// Created by Rvosuke on 2024/9/8.
//
#include "iostream"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/visualization/cloud_viewer.h"

int pcdReader() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::PointCloud<pcl::PointXYZ> cloud_out;

    pcl::io::loadPCDFile(R"(D:\LoRe\datasets\example.pcd)", cloud_blob);
    pcl::fromPCLPointCloud2(cloud_blob, *cloud);


    for (const auto &point: *cloud) {
        if (isnan(point.x) || isnan(point.y) || isnan(point.z))
            continue;
        cloud_out.push_back(point);
    }
    std::cerr << "Cloud size: " << cloud->size() << std::endl;
    std::cerr << "Cloud filtering size: " << cloud_out.size() << std::endl;
//    pcl::io::savePCDFileASCII(R"(D:\LoRe\datasets\example_out.pcd)", cloud_out);

    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {}
    return 0;
}