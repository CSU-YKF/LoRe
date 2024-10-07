//
// Created by Rvosuke on 2024/9/19.
//

#ifndef LORE_LORE_H
#define LORE_LORE_H

#include "view.h"
#include "type.h"
#include "filter.h"
#include "segmentation.h"
#include "fit.h"

#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>


int lore(const std::string& file_name);


#endif //LORE_LORE_H
