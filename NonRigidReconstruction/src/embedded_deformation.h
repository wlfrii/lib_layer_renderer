#ifndef EMBEDDED_DEFORMATION_H_LF
#define EMBEDDED_DEFORMATION_H_LF
#include <Eigen/Dense>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>


struct Vertex {
    Eigen::Vector3f coord;
    Eigen::Vector3f color;
    float weight;
};


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

    void addVertices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertices);

    void addVertices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertices,
                     const std::vector<
                        std::pair<pcl::PointXYZRGB, pcl::PointXYZRGB>
                            > &correspondences);

private:
    void updateKdTreeData();

    float _node_density;        // Density of ED nodes
    float _node_connectivity;   // The max node neighbors for each vertex

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _vertices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _ed_nodes;

    std::vector<Eigen::Matrix3f> _Rs; // rotation matrics
    std::vector<Eigen::Vector3f> _ts; // translations

    std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZRGB>> _kd_tree;
};

#endif // EMBEDDED_DEFORMATION_H_LF
