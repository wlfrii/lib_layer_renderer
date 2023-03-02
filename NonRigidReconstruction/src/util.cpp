#include "util.h"
#include <nanoflann.hpp>

namespace util {

struct PointWithOccurrences {
    PointWithOccurrences() : p({0, 0, 0}), n(0) {}
    Eigen::Vector3f p;  // The position of the point
    size_t          n;  // The number of occurrences
};
void voxelDownSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr in, float voxel_size,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    Eigen::Vector3f max_xyzf(0, 0, 0);
    Eigen::Vector3f min_xyzf(1000, 1000, 1000);
    for(size_t i = 0; i < in->size(); i++) {
        const pcl::PointXYZ& pt = in->at(i);
        if(max_xyzf[0] < pt.x) max_xyzf[0] = pt.x;
        if(max_xyzf[1] < pt.y) max_xyzf[1] = pt.y;
        if(max_xyzf[2] < pt.z) max_xyzf[2] = pt.z;
        if(min_xyzf[0] > pt.x) min_xyzf[0] = pt.x;
        if(min_xyzf[1] > pt.y) min_xyzf[1] = pt.y;
        if(min_xyzf[2] > pt.z) min_xyzf[2] = pt.z;
    }
    printf("x_range:[%f, %f]\n", min_xyzf[0], max_xyzf[0]);
    printf("y_range:[%f, %f]\n", min_xyzf[1], max_xyzf[1]);
    printf("z_range:[%f, %f]\n", min_xyzf[2], max_xyzf[2]);

    size_t x_node_num = floor((max_xyzf[0] - min_xyzf[0]) / voxel_size) + 1;
    size_t y_node_num = floor((max_xyzf[1] - min_xyzf[1]) / voxel_size) + 1;
    size_t z_node_num = floor((max_xyzf[2] - min_xyzf[2]) / voxel_size) + 1;
    printf("voxel size: %zux%zux%zu\n", x_node_num, y_node_num, z_node_num);

    // Group vertices into voxels
    std::vector<std::vector<std::vector<PointWithOccurrences>>> voxels(
                x_node_num, std::vector<std::vector<PointWithOccurrences>>(
                    y_node_num, std::vector<PointWithOccurrences>(
                        z_node_num, PointWithOccurrences())));
    for(size_t i = 0; i < in->size(); i++) {
        const pcl::PointXYZ& pt = in->at(i);
        int x_idx = floor(pt.x - min_xyzf[0] / voxel_size);
        int y_idx = floor(pt.y - min_xyzf[1] / voxel_size);
        int z_idx = floor(pt.z - min_xyzf[2] / voxel_size);

        //printf("index: %dx%dx%d\n", x_idx, y_idx, z_idx);

        voxels[x_idx][y_idx][z_idx].p += Eigen::Vector3f(pt.x, pt.y, pt.z);
        voxels[x_idx][y_idx][z_idx].n++;
    }

    // Filter out the valid voxels
    out->clear();
    for(size_t x_idx = 0; x_idx < x_node_num; x_idx++) {
        for(size_t y_idx = 0; y_idx < y_node_num; y_idx++) {
            for(size_t z_idx = 0; z_idx < z_node_num; z_idx++) {
                PointWithOccurrences& pt = voxels[x_idx][y_idx][z_idx];
                if(pt.n > 0) {
                    pt.p /= pt.n;
                    out->push_back({pt.p[0], pt.p[1], pt.p[2]});
                }
            }
        }
    }
}


void voxelDownSampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in, float voxel_size,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    Eigen::Vector3f max_xyzf(0, 0, 0);
    Eigen::Vector3f min_xyzf(1000, 1000, 1000);
    for(size_t i = 0; i < in->size(); i++) {
        const pcl::PointXYZRGB& pt = in->at(i);
        if(max_xyzf[0] < pt.x) max_xyzf[0] = pt.x;
        if(max_xyzf[1] < pt.y) max_xyzf[1] = pt.y;
        if(max_xyzf[2] < pt.z) max_xyzf[2] = pt.z;
        if(min_xyzf[0] > pt.x) min_xyzf[0] = pt.x;
        if(min_xyzf[1] > pt.y) min_xyzf[1] = pt.y;
        if(min_xyzf[2] > pt.z) min_xyzf[2] = pt.z;
    }
    printf("x_range:[%f, %f]\n", min_xyzf[0], max_xyzf[0]);
    printf("y_range:[%f, %f]\n", min_xyzf[1], max_xyzf[1]);
    printf("z_range:[%f, %f]\n", min_xyzf[2], max_xyzf[2]);

    size_t x_node_num = floor((max_xyzf[0] - min_xyzf[0]) / voxel_size) + 1;
    size_t y_node_num = floor((max_xyzf[1] - min_xyzf[1]) / voxel_size) + 1;
    size_t z_node_num = floor((max_xyzf[2] - min_xyzf[2]) / voxel_size) + 1;
    printf("voxel size: %zux%zux%zu\n", x_node_num, y_node_num, z_node_num);

    // Group vertices into voxels
    std::vector<std::vector<std::vector<PointWithOccurrences>>> voxels(
                x_node_num, std::vector<std::vector<PointWithOccurrences>>(
                    y_node_num, std::vector<PointWithOccurrences>(
                        z_node_num, PointWithOccurrences())));
    for(size_t i = 0; i < in->size(); i++) {
        const pcl::PointXYZRGB& pt = in->at(i);
        int x_idx = floor(pt.x - min_xyzf[0] / voxel_size);
        int y_idx = floor(pt.y - min_xyzf[1] / voxel_size);
        int z_idx = floor(pt.z - min_xyzf[2] / voxel_size);

        //printf("index: %dx%dx%d\n", x_idx, y_idx, z_idx);

        voxels[x_idx][y_idx][z_idx].p += Eigen::Vector3f(pt.x, pt.y, pt.z);
        voxels[x_idx][y_idx][z_idx].n++;
    }

    // Filter out the valid voxels
    out->clear();
    for(size_t x_idx = 0; x_idx < x_node_num; x_idx++) {
        for(size_t y_idx = 0; y_idx < y_node_num; y_idx++) {
            for(size_t z_idx = 0; z_idx < z_node_num; z_idx++) {
                PointWithOccurrences& pt = voxels[x_idx][y_idx][z_idx];
                if(pt.n > 0) {
                    pt.p /= pt.n;
                    out->push_back({pt.p[0], pt.p[1], pt.p[2]});
                }
            }
        }
    }
}



pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr estimateNormal(
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


pcl::PointCloud<pcl::PointNormal>::Ptr estimateNormal(
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


