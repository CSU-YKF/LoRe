//
// Created by Rvosuke on 2024/9/19.
//
#ifndef LORE_LORE_H
#define LORE_LORE_H

#include "print.h"
#include "type.h"
#include "filter.h"
#include "segmentation.h"

#include "pcl/io/pcd_io.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/visualization/cloud_viewer.h"


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
std::vector<float> computePointDistances(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

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
std::vector<float> geometryFitting(PointCloud &cloud_in, float param_distance);

int lore(int argc, char **argv);

#endif //LORE_LORE_H
