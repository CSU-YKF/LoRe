//
// Created by Rvosuke on 2024/9/8.
//

#include "../include/lore.h"


std::string lore(const std::string& file_name) {
    PointCloud::Ptr cloud_in(new PointCloud);
    PointCloud::Ptr circle_cloud(new PointCloud);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::io::loadPCDFile(file_name, *cloud_in);
    pcl::io::loadPCDFile("../datasets/circle_3d.pcd", *circle_cloud);
    std::ostringstream result_stream;

    result_stream << "Loaded " << cloud_in->size() << " data points from " << file_name << "\n";
    std::vector<float> box_min = {-450., 0., 1000.};
    std::vector<float> box_max = {750., 200., 1300.};
    cropBox(*cloud_in, box_min, box_max);
    downSampling(*cloud_in, 1);

    float avg_dist = 3;
    planarSegmentation(*cloud_in, avg_dist);
    std::vector<PointCloud::Ptr> cloud_interest = cloudClustering(*cloud_in, avg_dist);

    std::vector<std::vector<float>> results;
    for (const auto &cloud: cloud_interest) {
        downSampling(*cloud, avg_dist);
        if (icp_registration(cloud, circle_cloud)) {
            std::vector<float> result = geometryFitting(*cloud, avg_dist);
            results.push_back(result);
        }
    }
    float min_result = std::numeric_limits<float>::max();
    float max_result = std::numeric_limits<float>::min();
    std::vector<float> load_port;
    std::vector<float> recycle_port;
    for (const auto &res : results) {
        if (res[0] < min_result) {
            min_result = res[0];
            load_port = res;
        }
        if (res[0] > max_result) {
            max_result = res[0];
            recycle_port = res;
        }
    }
    result_stream << "加注口圆心: (" << load_port[0] << ", " << load_port[1] << ", " <<load_port[2] << ") \n";
    result_stream << "加注口法向: (" << load_port[3] << ", " << load_port[4] << ", " <<load_port[5] << ") \n";
    result_stream << "回收口圆心: (" << recycle_port[0] << ", " << recycle_port[1] << ", " <<recycle_port[2] << ") \n";
    result_stream << "回收口法向: (" << recycle_port[3] << ", " << recycle_port[4] << ", " <<recycle_port[5] << ") \n";

    return result_stream.str();
}
