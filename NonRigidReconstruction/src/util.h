#ifndef NRR_UTIL_H_LF
#define NRR_UTIL_H_LF
#include <Eigen/Dense>
#include <vector>
#include "point_cloud_handler.h"
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/io.h> // for concatenateFields
#include <opencv2/opencv.hpp>


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

void voxelDownSampling(pcl::PointCloud<pcl::PointNormal>::Ptr in, float voxel_size,
                       pcl::PointCloud<pcl::PointNormal>::Ptr out);

void voxelDownSampling(const Vertices& in, float voxel_size, Vertices& out);


void voxelDownSampling(const Vertices& in, float voxel_size,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr out);


/**
 * @brief Estimate normal for the given point cloud
 * @param pt_cloud
 * @return
 */
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr estimateNormal(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud);


pcl::PointCloud<pcl::PointNormal>::Ptr estimateNormal(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud);



void estimateImageNormal(const cv::Mat& gray_image, cv::Mat& normal_image);


/**
 * @brief Convert CV_32F/CV_64F to CV_8U
 * @param image
 * @return
 */
cv::Mat im2uchar(const cv::Mat & image);


void estimateTransform(
        const std::vector<
        std::pair<pcl::PointXYZ, pcl::PointXYZ>>& correspondences,
        Eigen::Matrix4f& transform);

void estimateTransform(
        const std::vector<
        std::pair<pcl::PointXYZ, pcl::PointXYZ>>& correspondences,
        Eigen::Matrix3f& R, Eigen::Vector3f& t);


} // namespace::util

#endif // NRR_UTIL_H_LF
