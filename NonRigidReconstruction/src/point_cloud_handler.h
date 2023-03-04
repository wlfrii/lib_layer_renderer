/**
 *
 * 2023.2.25
 */
#ifndef POINT_CLOUD_HANDLER_H_LF
#define POINT_CLOUD_HANDLER_H_LF
#include <nanoflann.hpp>
#include <Eigen/Dense>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h> // For PolygonMesh
#include <pcl/kdtree/kdtree_flann.h> // For KdTreeFLANN
#include <opencv2/opencv.hpp>
#include <lib_math/lib_math.h>
#include <lib_layer_renderer.h>


using PointCloudXYZ = Eigen::Matrix<float, Eigen::Dynamic, 3>;

namespace util {
void vectorPoints2pclPoints(const std::vector<Eigen::Vector3f>& in,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr out);


void eigenPoints2pclPoints(const PointCloudXYZ& in,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr out);

}


/**
 * @brief The PointCloudHandler class designed for handling the point cloud that
 * extracted from a stereo images.
 */
class PointCloudHandler
{
public:
    /**
     * @brief Constructor of PointCloudHandler
     * @param cam_proj The camera projector for binocular
     * @param cam_id  The used camera index, mmath::cam::LEFT/mmath::cam::RIGHT
     */
    PointCloudHandler(const std::shared_ptr<mmath::CameraProjector> cam_proj,
                      mmath::cam::ID cam_id);
    ~PointCloudHandler();


    /**
     * @brief Bind PointCloud (reconstructed from stereo images) to current
     * object
     * @param point_cloud
     */
    void bindPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);


    /**
     * @brief Bind PointCloud (reconstructed from stereo images) to current
     * object.
     * @param point_cloud
     */
    void bindPointCloud(const PointCloudXYZ& point_cloud);


    /**
     * @brief Bind texture for the binded point cloud
     * @param (in) texture  The left image in the stereo images
     * @param (in) cam_proj  The camera projector for binocular
     */
    void bindTexture(const cv::Mat& texture);


    /**
     * @brief Voxel downsampling current point cloud w.r.t specified voxel size
     * @param voxel_size
     */
    void voxelDownSampling(float voxel_size);


    /**
     * @brief Voxel downsampling current point cloud w.r.t specified voxel size
     * and assigned downsampled point cloud to output.
     * @param (in) voxel_size
     * @param (out) point_cloud
     */
    void voxelDownSampling(float voxel_size,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);


    /**
     * @brief Remove outliers by specifying a search radius and the minimum
     * number of neighbors in the search radius.
     * @param (in) radius
     * @param (in) min_neighbor_num
     */
    void rmOutliersByRadius(float radius, int min_neighbor_num);


    /**
     * @brief Remove outliers by specifying the number of neighbors for a point
     * and the maximum distance between a point to its neighbors.
     * @param (in) k
     * @param (in) max_neighbor_dis
     */
    void rmOutliersByKNeighbors(int k, float max_neighbor_dis);


    /**
     * @brief Remove outliers by statistical distance.
     * @param (in) k
     * @param (in) std_thresh
     */
    void statisticalOutliersRemoval(size_t k, float std_thresh);


    /**
     * @brief Create Mesh based on current point cloud
     * @param (in) search_radius  Max search radius
     * @param (in) mu  Max accepted distance to regard a point as neighbors
     * @param (in) max_neighbors  Max neighbor number
     * @param (out) vertices  Every three adjacent vertices consist a triangle
     */
    void createMesh(float search_radius, float mu, int max_neighbors,
                    std::vector<mlayer::Vertex3D> &vertices);


    /**
     * @brief Create vertices based on current point cloud
     * @param (out) vertices Each vertice is single 3D point
     */
    void createVertices(std::vector<mlayer::Vertex3D>& vertices);


    /**
     * @brief Get current point cloud
     * @return
     */
    const pcl::PointCloud<pcl::PointXYZ>::Ptr getCurrentPointCloud() const;


    /**
     * @brief Convert current point cloud to Eigen::Matrix
     * @return
     */
    Eigen::MatrixXf toEigenMatrix() const;



private:
    void updateKDTreeData();
    void readPointColor(const pcl::PointXYZ& pt, float& r, float& g, float& b);


    const std::shared_ptr<mmath::CameraProjector> _cam_proj;
    const mmath::cam::ID _cam_id;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _point_cloud;
    std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> _kd_tree;

    cv::Mat _texture;
};


#endif // POINT_CLOUD_HANDLER_H_LF
