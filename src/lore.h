//
// Created by zeyan on 2024/9/19.
//

#ifndef LORE_LORE_H
#define LORE_LORE_H
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/ModelCoefficients.h"

#include "pcl/search/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/visualization/cloud_viewer.h"
#include "print.h"
#include "filter.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

#endif //LORE_LORE_H
