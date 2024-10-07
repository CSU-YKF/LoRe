//
// Created by Rvosuke on 2024/9/8.
//

#include "../include/lore.h"


int lore(const std::string& file_name) {
    PointCloud::Ptr cloud_in(new PointCloud);
    PointCloud::Ptr circle_cloud(new PointCloud);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::io::loadPCDFile(file_name, *cloud_in);
    pcl::io::loadPCDFile("../datasets/circle_3d.pcd", *circle_cloud);
    printt("Loaded ", cloud_in->size(), " data points from ", file_name);

    std::vector<float> box_min = {-450, 0, 1000};
    std::vector<float> box_max = {750, 200, 1300};

    cropBox(*cloud_in, box_min, box_max);
    printt("Cloud after CropBox: ", cloud_in->size());
    downSampling(*cloud_in, 1);
//    normalEstimation(*cloud_in, normals);

//    std::vector<float> distances = computePointDistances(cloud_in);
//    float avg_dist = distances[2] * 3;
    float avg_dist = 3;
    printt("Average distance: ", avg_dist);
    planarSegmentation(*cloud_in, avg_dist);
    printt("Cloud after outlier removal: ", cloud_in->size());

    std::vector<PointCloud::Ptr> cloud_interest = cloudClustering(*cloud_in, avg_dist);
    for (const auto &cloud: cloud_interest) {
        downSampling(*cloud, avg_dist);
        if (icp_registration(cloud, circle_cloud)) {
            std::vector<float> result = geometryFitting(*cloud, avg_dist);
            printt("Circle center: ", result[0], " ", result[1], " ", result[2]);
            printt("Circle normal: ", result[3], " ", result[4], " ", result[5]);
        }
    }
    return 0;
}
