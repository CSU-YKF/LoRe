//
// Created by Rvosuke on 2024/10/7.
//

#ifndef LORE_OPTIMIZE_H
#define LORE_OPTIMIZE_H

#include "type.h"

#include <vector>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>


/**
 * @brief Computes the distance from each point in the point cloud to its nearest neighbor,
 *        and returns the minimum, maximum, and average distances.
 *
 * This function iterates through each point in the input point cloud and uses a KD-Tree
 * to find the nearest neighbor for each point. It then calculates the distance to the
 * nearest neighbor and updates the minimum, maximum, and total distances accordingly.
 * Finally, it computes the average distance and returns all three distances in a vector.
 *
 * @param cloud The input point cloud.
 * @return A vector containing the minimum distance, maximum distance, and average distance.
 */
std::vector<float> computePointDistances(const PointCloud::Ptr &cloud);

/**
 * @brief Estimates the normals of the input point cloud.
 *
 * This function estimates the normals of the input point cloud using the normal estimation
 * method provided by the Point Cloud Library (PCL). It uses a KD-Tree to search for the
 * nearest neighbors of each point and computes the normals based on the neighbors.
 *
 * @param cloud_in The input point cloud.
 * @param normals The output normals.
 */
void normalEstimation(PointCloud &cloud_in, pcl::PointCloud<pcl::Normal>::Ptr &normals);


#endif //LORE_OPTIMIZE_H
