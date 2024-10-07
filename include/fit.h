//
// Created by Rvosuke on 2024/10/7.
//

#ifndef LORE_FIT_H
#define LORE_FIT_H

#include "type.h"
#include "view.h"

#include <vector>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/icp.h>


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
std::vector<float> geometryFitting(PointCloud &cloud_in, float param_distance = 3.);

/**
 * @brief Perform ICP registration to evaluate the fitting.
 *
 * This function uses the Iterative Closest Point (ICP) algorithm to align the source point cloud
 * to the target point cloud. It sets the input source and target clouds, performs the alignment,
 * and checks if the ICP algorithm has converged and if the fitness score is below a specified threshold.
 *
 * @param source_cloud The source point cloud to be aligned.
 * @param target_cloud The target point cloud to align to.
 * @return True if the ICP algorithm has converged and the fitness score is below the threshold, false otherwise.
 */
bool icp_registration(const PointCloud::Ptr& source_cloud, const PointCloud::Ptr& target_cloud, float threshold = 100);


#endif //LORE_FIT_H
