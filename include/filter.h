//
// Created by Rvosuke on 2024/9/19.
//
#ifndef LORE_FILTER_H
#define LORE_FILTER_H

#include "print.h"
#include "type.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>


/**
 * @brief Filters the point cloud data to remove noise and sparse points.
 * Steps:
 * - Downsampling: Uses the Voxel Grid Downsampling algorithm to reduce the size of the point cloud.
 * - Outlier Removal: Uses either Statistical Outlier Removal or Voxel Grid Downsampling to reduce noise and density, optimizing processing speed.
 * - Note: Voxel Grid Downsampling is used instead of Statistical Outlier Removal due to its faster processing speed.
 * @param cloud_in Input point cloud
 * @param param_distance Filtering parameter
 */
void downSampling(PointCloud &cloud_in, float param_distance = 1.0);

/**
 * @brief Removes ground or irrelevant points using a plane segmentation algorithm (e.g., RANSAC), retaining only points related to the filling and recovering ports.
 * @param cloud_in Input point cloud
 * @param param_distance Filtering parameter
 */
void removeOutlier(PointCloud &cloud_in, float param_distance);

/**
 * @brief CropBox-like filters out the point cloud data within a given cube of the user.
 * @param cloud_in The input point cloud to be clipped.
 */
void cropBox(PointCloud &cloud_in, std::vector<float> &box_min, std::vector<float> &box_max);

#endif //LORE_FILTER_H
