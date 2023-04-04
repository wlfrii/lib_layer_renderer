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
    EmbeddedDeformation(const std::shared_ptr<mmath::CameraProjector> cam_proj,
                        const mmath::cam::ID cam_id,
                        float vertex_density,
                        float node_density = 4.f, int node_connectivity = 6);
    ~EmbeddedDeformation();

    void addVertices(const Vertices& new_vertices);

    void addVertices(const Vertices& new_vertices,
                     const cv::Mat& depthmap, const cv::Mat& texture,
                     const std::vector<
                        std::pair<pcl::PointXYZ, pcl::PointXYZ>
                            >& correspondences);

    const Vertices& getVertices() const;

    void projectPointCloud(float search_radius, float mu, int max_neighbors,
                           std::vector<mlayer::Vertex3D>& vertices);
    void projectPointCloud(std::vector<mlayer::Vertex3D>& vertices);

private:
    void regenerateNodes();


    /**
     * @brief depthImageRegistration
     * @param R  The rotation from previous camera frame to current
     * @param t  The translation from previous camera frame to current
     * @param depthmap
     * @param visible_points
     */
    void depthImageRegistration(
            const Eigen::Matrix3f& R, const Eigen::Vector3f& t,
            const cv::Mat& depthmap, Vertices& visible_points);


    void fusionVertices(const Vertices &new_vertices,
                        const cv::Mat& depthmap, const cv::Mat& texture);


    const std::shared_ptr<mmath::CameraProjector> _cam_proj;
    const mmath::cam::ID _cam_id;

    float _vertex_density;
    float _node_density;        // Density of ED nodes
    int   _node_connectivity;   // The max node neighbors for each vertex
    int   _K;                   // Number of neighbors
    float _neighbor_dis_thresh; // Maximum distance for a node to be a neighbor

    Vertices _vertices;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _nodes;
    std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> _kd_tree;

    /* In this project, Global Rt is w.r.t the first image, and denotes the
     * camera movement.
     */
//    Eigen::Matrix3f _R; // Global R
//    Eigen::Vector3f _t; // Global t;

    size_t   _timestamp;       // Tag each group of new vertices
};

#endif // EMBEDDED_DEFORMATION_H_LF
