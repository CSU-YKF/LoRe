#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/box_clipper3D.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
using namespace std;

/**
 * 类BoxClipper3D实现用一个以原点为中心、XYZ各个方向尺寸为2、经过用户指定仿射变换的立方体进行"空间裁剪",
 * 通过设置一个仿射变换矩阵先对立方体进行"变换处理",之后输出仿射变换后落在该立方体内的点集。
 *
 * @param cloud_in The input point cloud to be clipped.
 */
static void boxclip3d(pcl::PointCloud<pcl::PointXYZ>& cloud_in)
{
    Eigen::Affine3f transformation = Eigen::Affine3f::Identity();

    // Find the centroid of the point cloud
    Eigen::Vector4f center;
    pcl::compute3DCentroid(cloud_in, center);
    cout << center.x() << "," << center.y() << "," << center.z() << endl;
    // Move the centroid of the point cloud to the origin
	// 这里有个疑问，为什么要将点云的中心移动到原点？ 以及为什么要将z轴移动到-1？
    transformation.translation() << -center.x(), -center.y(), -center.z() - 1;

    // Create a BoxClipper3D object with the transformation
    pcl::BoxClipper3D<pcl::PointXYZ> boxClip3d(transformation);
    pcl::Indices cliped;
    boxClip3d.clipPointCloud3D(cloud_in, cliped);

    cout << cliped.size() << endl;
    // Change the color of the filtered points
    //for (auto& idx : cliped)
    //{
    //    cloud_in.points[idx].r = 255;
    //    cloud_in.points[idx].g = 0;
    //}
}

/**
 * 类PlaneClipper3D在三维空间实现平面裁剪。
 *
 * @param cloud_in The input point cloud to be clipped.
 */
static void planeClip3D(pcl::PointCloud<pcl::PointXYZRGB>& cloud_in)
{
    // Set the clipping plane
    // The constructor takes an Eigen::Vector4f as the homogeneous representation of the plane.
    Eigen::Vector4f plane(20, 20, -1, 1);
    pcl::PlaneClipper3D<pcl::PointXYZRGB> planeClip(plane);

    // Clip the point cloud using PlaneClipper3D
    pcl::Indices clipped;
    planeClip.clipPointCloud3D(cloud_in, clipped);

    // Change the color of the filtered points
    for (const auto& idx : clipped)
    {
        cloud_in.points[idx].r = 0;
        cloud_in.points[idx].b = 0;
    }

    cout << clipped.size() << endl;
}

/**
 * 类CropBox过滤掉在用户给定立方体内的点云数据。
 *
 * @param cloud_in The input point cloud to be clipped.
 */
static void cropBox(pcl::PointCloud<pcl::PointXYZRGB>& cloud_in)
{
    Eigen::Vector4f center;
    pcl::compute3DCentroid(cloud_in, center);
    cout << center.x() << "," << center.y() << "," << center.z() << endl;
    // Set the filtering box
    pcl::CropBox<pcl::PointXYZRGB> cropbox;
    cropbox.setInputCloud(cloud_in.makeShared());
    cropbox.setMin(Eigen::Vector4f(-0.5, 0, 1.5, 1)); // Set the minimum point
    cropbox.setMax(Eigen::Vector4f(0, 0.5, 2, 1)); // Set the maximum point

    // Crop the point cloud using CropBox
    pcl::Indices clipped;
    cropbox.filter(clipped);

    // Change the color of the filtered points
    for (const auto& idx : clipped)
    {
        cloud_in.points[idx].r = 0;
        cloud_in.points[idx].b = 0;
    }
    cout << clipped.size() << endl;
}

/**
 * 类CropBox过滤在给定三维封闭曲面或二维封闭多边形内部或外部的点云数据,
 * 封闭曲面或多边形由类ConvexHull或ConcaveHull 处理产生。
 *
 * @param cloud_in The input point cloud to be clipped.
 */
static void cropHull(pcl::PointCloud<pcl::PointXYZRGB>& cloud_in)
{
    Eigen::Vector4f center;
    pcl::compute3DCentroid(cloud_in, center);

    Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
    transformation.translation() << -center.x(), -center.y(), -center.z();
    pcl::transformPointCloud<pcl::PointXYZRGB>(cloud_in, cloud_in, transformation);

    // Define a 2D point cloud representing the bounding box
    pcl::PointCloud<pcl::PointXYZRGB> boundingbox_ptr;
    boundingbox_ptr.push_back(pcl::PointXYZRGB(10, 10, 0, 255, 0, 0));
    boundingbox_ptr.push_back(pcl::PointXYZRGB(10, -10, 0, 255, 0, 0));
    boundingbox_ptr.push_back(pcl::PointXYZRGB(-10, 10, 0, 255, 0, 0));
    boundingbox_ptr.push_back(pcl::PointXYZRGB(-10, -10, 0, 255, 0, 0));
    boundingbox_ptr.push_back(pcl::PointXYZRGB(15, 10, 0, 255, 0, 0));

    pcl::ConvexHull<pcl::PointXYZRGB> hull; // Create a convex hull object
    hull.setInputCloud(boundingbox_ptr.makeShared()); // Set the input point cloud
    hull.setDimension(2); // Set the dimension of the convex hull
    std::vector<pcl::Vertices> polygons; // Vector to store the vertices of the convex hull
    pcl::PointCloud<pcl::PointXYZRGB> surface_hull; // Point cloud used to describe the shape of the convex hull
    hull.reconstruct(surface_hull, polygons); // Compute the 2D convex hull

    pcl::CropHull<pcl::PointXYZRGB> bb_filter; // Create a CropHull object
    bb_filter.setDim(2); // Set the dimension: it should match the dimension of the convex hull
    bb_filter.setInputCloud(cloud_in.makeShared()); // Set the point cloud to be filtered
    bb_filter.setHullIndices(polygons); // Set the indices of the closed polygon
    bb_filter.setHullCloud(surface_hull.makeShared()); // Set the shape of the closed polygon
    pcl::Indices clipped;
    bb_filter.filter(clipped); // Perform the CropHull filtering and store the result in clipped

    cout << clipped.size() << endl;
    for (auto& idx : clipped)
    {
        cloud_in.points[idx].r = 255;
        cloud_in.points[idx].g = 0;
    }
}

//int main(int argc, char* argv[])
//{
//    // Load the point cloud
//    pcl::PCLPointCloud2 cloud;
//
//    pcl::io::loadPCDFile("D:\\LoRe\\datasets\\实车点云\\ICP_input0.pcd", cloud);
//    pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
//
//    pcl::fromPCLPointCloud2(cloud, cloud_in);
//
//    // Clipper
//    //boxclip3d(cloud_in);
//    //planeClip3D(cloud_in);
//    //cropBox(cloud_in);
//    //cropHull(cloud_in);
//
//    // Create a viewer window
//    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
//    viewer->setWindowName("BoxClipper3D");
//
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_in.makeShared());
//    viewer->addPointCloud(cloud_in.makeShared(), rgb);
//    //viewer->addCube(-0.5,0,0,0.5,1.5,2,1.0,0.0,0.0);
//    //viewer->addPolygon<pcl::PointXYZRGB>(boundingbox_ptr,0,1.0,0);
//    viewer->setRepresentationToWireframeForAllActors();
//
//    while (!viewer->wasStopped())
//        viewer->spinOnce(100);
//
//    return 0;
//}
//
//int main(int argc, char** argv)
//{
//    // 创建一个PointCloud对象
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//    // 读取PCD文件
//    if (pcl::io::loadPCDFile<pcl::PointXYZ>(R"(D:\LoRe\datasets\example.pcd)", *cloud) == -1) // 这里替换为你的PCD文件路径
//    {
//        PCL_ERROR("Couldn't read file example.pcd \n");
//        return (-1);
//    }
//
//    std::cout << "Loaded "
//        << cloud->width * cloud->height
//        << " data points from example.pcd with the following fields: "
//        << std::endl;
//
//	// 打印点云数据范围
//	pcl::PointXYZ minPt, maxPt;
//	pcl::getMinMax3D(*cloud, minPt, maxPt);
//    std::cout << "Min point: (" << minPt.x << ", " << minPt.y << ", " << minPt.z << ")" << std::endl;
//    std::cout << "Max point: (" << maxPt.x << ", " << maxPt.y << ", " << maxPt.z << ")" << std::endl;
//
//    boxclip3d(*cloud);
//
//    // 显示点云数据
//    pcl::visualization::CloudViewer viewer("Cloud Viewer");
//    viewer.showCloud(cloud);
//
//    // 保持窗口打开，直到用户关闭
//    while (!viewer.wasStopped())
//    {
//    }
//
//    return 0;
//}
