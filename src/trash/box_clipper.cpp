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
#include <pcl/console/parse.h>
using namespace std;

/**
 * 类BoxClipper3D实现用一个以原点为中心、XYZ各个方向尺寸为2、经过用户指定仿射变换的立方体进行"空间裁剪",
 * 通过设置一个仿射变换矩阵先对立方体进行"变换处理",之后输出仿射变换后落在该立方体内的点集。
 *
 * @param cloud_in The input point cloud to be clipped.
 */
static void boxClip3d(pcl::PointCloud<pcl::PointXYZ>& cloud_in)
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
    pcl::BoxClipper3D<pcl::PointXYZ> box_clip3d(transformation);
    pcl::Indices clipped;
    box_clip3d.clipPointCloud3D(cloud_in, clipped);

    cout << clipped.size() << endl;
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
