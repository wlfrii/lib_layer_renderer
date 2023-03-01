#ifndef POINT_CLOUD_HANDLER_H_LF
#define POINT_CLOUD_HANDLER_H_LF
#include <nanoflann.hpp>
#include <Eigen/Dense>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h> // For PolygonMesh
#include <pcl/kdtree/kdtree_flann.h> // For KdTreeFLANN


using PointCloudXYZ = Eigen::Matrix<float, Eigen::Dynamic, 3>;

namespace util {
void vectorPoints2pclPoints(const std::vector<Eigen::Vector3f>& in,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr out);


void eigenPoints2pclPoints(const PointCloudXYZ& in,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr out);

}


class PointCloudHandler
{
public:
    PointCloudHandler();
    ~PointCloudHandler();

    void bindPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);


    void voxelDownSampling(float voxel_size);


    void voxelDownSampling(float voxel_size,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);


    void rmOutliersByRadius(float radius, int min_neighbor_num);


    void rmOutliersByKNeighbors(int k, float max_neighbor_dis);


    void statisticalOutliersRemoval(size_t k, float std_thresh);


    const pcl::PointCloud<pcl::PointXYZ>::Ptr getCurrentPointCloud() const;


    void createMesh(pcl::PolygonMesh& mesh, float search_radius, float mu,
                    int max_neighbors);


private:
    void updateKDTreeData();


//    PointCloudXYZ _point_cloud;
//    std::shared_ptr<nanoflann::KDTreeEigenMatrixAdaptor<PointCloudXYZ>> _kd_tree;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _point_cloud;
    std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> _kd_tree;
};


#endif // POINT_CLOUD_HANDLER_H_LF
