#ifndef NRR_UTIL_H_LF
#define NRR_UTIL_H_LF
#include <Eigen/Dense>
#include <vector>
#include "point_cloud_handler.h"
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/io.h> // for concatenateFields

namespace util {

/**
 * @brief Estimate normal for the given point cloud
 * @param pt_cloud
 * @return
 */
inline pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr estimateNormal(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
                new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(pt_cloud);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(pt_cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(30);

    // Estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(
                new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);
    //printf("normal.size:%zu\n", normals->size());

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(
                new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    //* cloud_with_normals = cloud + normals
    pcl::concatenateFields(*pt_cloud, *normals, *cloud_with_normals);
    //printf("cloud_with_normals.size:%zu\n", cloud_with_normals->size());

    return cloud_with_normals;
}

inline pcl::PointCloud<pcl::PointNormal>::Ptr estimateNormal(
        pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
                new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pt_cloud);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(pt_cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(30);

    // Estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(
                new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);
    //printf("normal.size:%zu\n", normals->size());

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
                new pcl::PointCloud<pcl::PointNormal>);
    //* cloud_with_normals = cloud + normals
    pcl::concatenateFields(*pt_cloud, *normals, *cloud_with_normals);
    //printf("cloud_with_normals.size:%zu\n", cloud_with_normals->size());

    return cloud_with_normals;
}


} // namespace::util

#endif // NRR_UTIL_H_LF
