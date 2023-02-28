#ifndef POINT_CLOUD_HANDLER_H_LF
#define POINT_CLOUD_HANDLER_H_LF
#include <nanoflann.hpp>
#include <Eigen/Dense>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h> // For PolygonMesh
#include <pcl/kdtree/kdtree_flann.h> // For KdTreeFLANN


// Smoothing depth map
// https://www.cnblogs.com/zzk0/p/10468502.html

using PointCloudXYZ = Eigen::Matrix<float, Eigen::Dynamic, 3>;

namespace util {
void vectorPoints2EigenPoints(const std::vector<Eigen::Vector3f>& in,
                              PointCloudXYZ& out);


void eigenPoints2vectorPoints(const PointCloudXYZ& in,
                              std::vector<Eigen::Vector3f>& out);


void pclPoints2EigenPoints(const pcl::PointCloud<pcl::PointNormal>::Ptr in,
                           PointCloudXYZ& out);
}


class PointCloudHandler
{
public:
    PointCloudHandler(const PointCloudXYZ& point_cloud);
    PointCloudHandler(const std::vector<Eigen::Vector3f>& point_cloud);
    ~PointCloudHandler();

    void voxelDownSampling(float voxel_size);


    void rmOutliersByRadius(float radius, int min_neighbor_num);


    void rmOutliersByKNeighbors(int k, float max_neighbor_dis);


    const PointCloudXYZ& getCurrentPointCloud() const;


    std::vector<size_t> findKNeighbors(const Eigen::Vector3f& query_point, int k);


    std::vector<size_t> findNeighborsByRadius(const Eigen::Vector3f& query_point,
                                              float radius);

    pcl::PointCloud<pcl::PointXYZ>::Ptr toPCLPointCloud() const;


    void createMesh(pcl::PolygonMesh& mesh, float search_radius, float mu,
                    int max_neighbors, bool use_mls = true);


private:
    void setKDTree();


    PointCloudXYZ _point_cloud;
    std::shared_ptr<nanoflann::KDTreeEigenMatrixAdaptor<PointCloudXYZ>> _kd_tree;
    std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> _kdtree;
};


#endif // POINT_CLOUD_HANDLER_H_LF
