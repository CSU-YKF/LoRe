//
// Created by Rvosuke on 2024/9/8.
//
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/visualization/cloud_viewer.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

/**
 * @brief 对点云数据进行滤波处理，去除噪声和稀疏点。
 * 步骤:
 * 下采样：使用体素网格下采样算法（Voxel Grid Down sampling）对点云进行降采样，减少点云数据量。
 * 去除离群点：使用统计滤波（Statistical Outlier Removal）或体素网格下采样（Voxel Grid Down sampling）来降低点云的噪声和密度，优化处理速度。
 * 由于Statistical Outlier Removal算法的速度太慢，因此使用Voxel Grid Down sampling算法。
 * @param cloud_in 输入点云
 * @param param_distance 滤波参数
 */
static void downSampling(PointCloud &cloud_in, float param_distance = 1.0) {
    //    pcl::StatisticalOutlierRemoval<PointT> sor;
    //    sor.setInputCloud(cloud_in.makeShared());
    //    sor.setMeanK(50);
    //    sor.setStddevMulThresh(1.0);
    //    sor.filter(cloud_in);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud_in.makeShared());
    vg.setLeafSize(3 * param_distance, 3 * param_distance, 3 * param_distance);
    vg.filter(cloud_in);
    std::cout << "Cloud after Voxel Grid: " << cloud_in.size() << std::endl;
}

/**
 * @brief 使用平面分割算法（如RANSAC）将地面或不相关点剔除，仅保留与加注口和回收口相关的点。
 * @param cloud_in 输入点云
 * @param param_distance 滤波参数
 */
static void removeOutlier(PointCloud &cloud_in, float param_distance) {
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
        std::cout << "After remove planar component: " << cloud_in.size() << std::endl;
    }
}


/**
 * @brief 将加注口和回收口从点云数据中分离出来，以便进一步拟合其位姿。
 * 步骤:
 * 点云聚类：使用Euclidean Cluster Extraction算法对点云进行聚类，将罐车的不同结构（如加注口、回收口、罐体等）分离成不同的点云簇。
 * 感兴趣区域提取：根据罐车的结构和已知几何布局，对位于加注口和回收口区域的点云进行提取。
 * 此时可结合RGB图像进行辅助，如果图像质量较好，可以使用颜色信息增强分割精度。
 *
 * @param cloud_in 输入点云
 * @param param_distance 滤波参数
 * @param point_size 点云簇的最小和最大尺寸
 */
static std::vector<PointCloud::Ptr> segmentation(PointCloud &cloud_in, float param_distance, int point_size) {
    PointCloud::Ptr cloud = cloud_in.makeShared();
    // 点云聚类
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(3 * param_distance);
    // min cluster size 比较重要，如果设置太小，会导致一些噪声点被误认为是簇；如果设置太大，会导致无法检测到目标。
    ec.setMinClusterSize(point_size / 100);
    ec.setMaxClusterSize(point_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    std::cout << "Number of clusters: " << cluster_indices.size() << std::endl;

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
        std::cout << "Cluster " << cluster_id << " size: " << cloud_cluster->size() << std::endl;
        cluster_id++;
        clusters.push_back(cloud_cluster);
    }

    return clusters;
}


/**
 * @brief 计算点云中每个点到最近邻点的距离，得到最小距离、最大距离和平均距离。
 */
std::vector<float> computePointDistances(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    float min_dist, max_dist, avg_dist = 0;
    // 初始化变量
    min_dist = std::numeric_limits<float>::max();
    max_dist = 0.0;
    float total_dist = 0.0;
    int point_count = 0;

    // 使用KD-Tree加速最近邻搜索
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    // 遍历点云中的每个点
    for (auto point: cloud->points) {
        std::vector<int> pointIdxNKNSearch(2); // 搜索最近的两个点（包括自身）
        std::vector<float> pointNKNSquaredDistance(2);

        if (tree.nearestKSearch(point, 2, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            float dist = std::sqrt(pointNKNSquaredDistance[1]); // 最近邻距离

            // 更新最小值、最大值
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;

            total_dist += dist;
            point_count++;
        }
    }

    // 计算平均距离
    if (point_count > 0) {
        avg_dist = total_dist / float(point_count);
    }

    return {min_dist, max_dist, avg_dist};
}


/**
 * @brief Perform geometric fitting (fit circular structures) on the point cloud data of the filling and recovering ports, to obtain their pose.
 * Steps:
 * Circle Fitting: Use the RANSAC or Least Squares Circle Fitting algorithm in PCL, to extract the circular edges of the filling and recovering ports from the point cloud and fit the center position and normal vector.
 * RANSAC is suitable for data with noise, as it can iteratively select random point sets for fitting and find the optimal solution.
 * Normal Vector Estimation: Calculate the normal vector from the point set fitted to the circle.
 * If the circular surface is uneven or has deviations, the direction of the normal vector can be determined in conjunction with Principal Component Analysis (PCA).
 *
 * @param cloud_in Input point cloud
 * @return Outputs the 3D coordinates of the circle center $(x, y, z)$ and the normal vector $(n_x, n_y, n_z)$.
 */
static std::vector<float> geometryFitting(PointCloud &cloud_in, float param_distance) {
    // 圆拟合
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

    int inside_circle_count = 0;
    // 提取圆的参数：圆心 (coefficients->values[0], coefficients->values[1], coefficients->values[2])
    // 半径 coefficients->values[3]
    double cx = coefficients->values[0];
    double cy = coefficients->values[1];
    double cz = coefficients->values[2];
    double radius = (coefficients->values[3] + coefficients->values[4] + coefficients->values[5]) / 3;

    // 遍历点云，计算每个点到圆心的距离
    for (const auto &point: cloud_in.points) {
        double distance_to_center = std::sqrt((point.x - cx) * (point.x - cx) +
                                              (point.y - cy) * (point.y - cy));

        // 判断该点是否在圆内部
        if (distance_to_center <= radius) {
            inside_circle_count++;
        }
    }

    // 计算圆内部点的比例
    double inside_ratio = static_cast<double>(inside_circle_count) / cloud_in.size();


    if (inside_ratio > 0.9) {
        // 法向量估计
        Eigen::Vector3f center(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        Eigen::Vector3f normal(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
        std::cout << "Circle center: " << center << std::endl;
        std::cout << "Circle normal: " << normal << std::endl;
        return {center.x(), center.y(), center.z(), normal.x(), normal.y(), normal.z()};
    }

    std::cerr << "Circle fitting failed!" << std::endl;
    return {};
}


int main() {
    PointCloud::Ptr cloud_in(new PointCloud);
    PointCloud::Ptr cloud_load(new PointCloud);
    PointCloud::Ptr cloud_recycle(new PointCloud);
    pcl::io::loadPCDFile(R"(D:\LoRe\datasets\source.pcd)", *cloud_in);
    pcl::io::loadPCDFile(R"(D:\LoRe\datasets\sor_pass_ly.pcd)", *cloud_load);
    pcl::io::loadPCDFile(R"(D:\LoRe\datasets\sor_pass_ry.pcd)", *cloud_recycle);

//    pcl::visualization::CloudViewer viewer("Cloud Viewer"), viewer2("Cloud Viewer");
//    viewer.showCloud(cloud_load);
//    while (!viewer.wasStopped()) {}
//    viewer2.showCloud(cloud_recycle);
//    while (!viewer2.wasStopped()) {}

    std::vector<int> indices;
    std::cout << "Cloud before filtering: " << cloud_in->size() << std::endl;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
    pcl::removeNaNFromPointCloud(*cloud_load, *cloud_load, indices);
    pcl::removeNaNFromPointCloud(*cloud_recycle, *cloud_recycle, indices);
    std::cout << "Cloud after removing NaN: " << cloud_in->size() << std::endl;

    downSampling(*cloud_in);
    std::vector<float> distances = computePointDistances(cloud_in);
    float avg_dist = distances[2];
    std::cout << "Average distance after down sampling: " << avg_dist << std::endl;

    removeOutlier(*cloud_in, avg_dist);
    distances = computePointDistances(cloud_in);
    avg_dist = distances[2];
    std::cout << "Average distance after RANSAC: " << avg_dist << std::endl;

    std::vector<PointCloud::Ptr> cloud_interest = segmentation(*cloud_in, avg_dist, cloud_in->size());
    for (const auto &cloud: cloud_interest) {
        pcl::visualization::CloudViewer viewer3("Cloud Viewer");
        viewer3.showCloud(cloud);
        while (!viewer3.wasStopped()) {}
        // 进行几何拟合
        std::vector<float> result = geometryFitting(*cloud, avg_dist);
        std::cout << result[3] << result[4] << result[5] << std::endl;
    }

    return 0;
}