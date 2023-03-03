#ifndef NRR_UTIL_H_LF
#define NRR_UTIL_H_LF
#include <Eigen/Dense>
#include <vector>
#include "point_cloud_handler.h"
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/io.h> // for concatenateFields

namespace util {

inline float distance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
    return sqrtf(powf(p1.x - p2.x, 2) + powf(p1.y - p2.y, 2) +
                 powf(p1.z - p2.z, 2));
}


inline Eigen::VectorXd nodeWeights(const std::vector<float>& squared_distances){
    Eigen::VectorXd weight(squared_distances.size());
    float sum = 0;
    float dmax = squared_distances.back();
    for(size_t j = 0; j < squared_distances.size(); j++) {
        weight[j] = 1 - squared_distances[j] / dmax;
        sum += weight[j];
    }
    weight /= sum;
    return weight;
}


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
