//
// Created by Rvosuke on 2024/9/24.
//
#ifndef LORE_SEGMENTATION_H
#define LORE_SEGMENTATION_H

#include "type.h"
#include "print.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iomanip> // for setw, setfill


/**
* @brief Segregate the filling and recovering ports from the point cloud data for further pose fitting.
* Steps:
* Point Cloud Clustering: Use the Euclidean Cluster Extraction algorithm to cluster the point cloud, separating different structures of the tank truck (e.g., filling port, recovering port, tank body) into different point cloud clusters.
* Region of Interest Extraction: Based on the structure and known geometric layout of the tank truck, extract the point cloud located in the regions of the filling and recovering ports.
* Optionally, if the quality of the RGB images is good, use color information to enhance segmentation accuracy.
*
* @param cloud_in Input point cloud
* @param param_distance Filtering parameter
* @param point_size Minimum and maximum size of the point cloud clusters
*/
std::vector<PointCloud::Ptr> segmentation(PointCloud &cloud_in, float param_distance, int point_size);

#endif //LORE_SEGMENTATION_H
