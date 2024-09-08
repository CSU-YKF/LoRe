//
// Created by Rvosuke on 2024/9/8.
//
#include "iostream"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_types.h"

int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile(R"(D:\LoRe\datasets\examples.pcd)", cloud_blob);
    pcl::fromPCLPointCloud2(cloud_blob, *cloud);

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    sor.filter(*cloud_filtered);

    pcl::visualization::CloudViewer viewer("Voxel Grid");
    viewer.showCloud(cloud_filtered);
    while(!viewer.wasStopped()) {}

    return 0;
}