#ifndef NRR_UTIL_H_LF
#define NRR_UTIL_H_LF
#include <Eigen/Dense>
#include <vector>
#include "point_cloud_handler.h"
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/io.h> // for concatenateFields

namespace util {

void voxelDownSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr in, float voxel_size,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr out);

void voxelDownSampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in, float voxel_size,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr out);


/**
 * @brief Estimate normal for the given point cloud
 * @param pt_cloud
 * @return
 */
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr estimateNormal(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud);


pcl::PointCloud<pcl::PointNormal>::Ptr estimateNormal(
        pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud);


} // namespace::util

#endif // NRR_UTIL_H_LF
