#ifndef EMBEDDED_DEFORMATION_H_LF
#define EMBEDDED_DEFORMATION_H_LF
#include <Eigen/Dense>
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>
#include "point_cloud_define.h"
#include <lib_math/lib_math.h>
#include <lib_layer_renderer.h>
#include <opencv2/opencv.hpp>


#define USE_ED_SHOW 1

/**
 * @brief The EmbeddedDeformation class is designed for parameterize the norigid
 * deformation field.
 *
 */
class EmbeddedDeformation
{
public:
    EmbeddedDeformation(float node_density = 5.f, int node_connectivity = 6);
    ~EmbeddedDeformation();

    void addVertices(const Vertices& new_vertices);

    void addVertices(const Vertices& new_vertices, const cv::Mat& depthmap,
                     const std::vector<
                        std::pair<pcl::PointXYZ, pcl::PointXYZ>
                            >& correspondences);

    const Vertices& getVertices() const;

    void projectPointCloud(float search_radius, float mu, int max_neighbors,
                           std::vector<mlayer::Vertex3D>& vertices);

private:
    void updateKdTreeData();

    void depthImageRegistration(const cv::Mat& depthmap,
            pcl::PointCloud<pcl::PointNormal>::Ptr visible_points);


    const std::shared_ptr<mmath::CameraProjector> _cam_proj;

    float _node_density;        // Density of ED nodes
    float _node_connectivity;   // The max node neighbors for each vertex

    Vertices _vertices;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _nodes;

    std::vector<Eigen::Matrix3d> _Rs; // rotation matrics
    std::vector<Eigen::Vector3d> _ts; // translations

    std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> _kd_tree;
};

#endif // EMBEDDED_DEFORMATION_H_LF
