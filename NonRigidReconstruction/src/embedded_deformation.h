#ifndef EMBEDDED_DEFORMATION_H_LF
#define EMBEDDED_DEFORMATION_H_LF
#include <Eigen/Dense>
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>


struct Vertices{
    pcl::PointCloud<pcl::PointXYZ>::Ptr coords;
    std::vector<Eigen::Vector3f> colors;
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

    void addVertices(pcl::PointCloud<pcl::PointXYZ>::Ptr coords,
                     const std::vector<Eigen::Vector3f>& colors);

    void addVertices(pcl::PointCloud<pcl::PointXYZ>::Ptr coords,
                     const std::vector<Eigen::Vector3f>& colors,
                     const std::vector<
                        std::pair<pcl::PointXYZ, pcl::PointXYZ>
                            > &correspondences);

    const Vertices& getVertices() const;

private:
    void updateKdTreeData();

    float _node_density;        // Density of ED nodes
    float _node_connectivity;   // The max node neighbors for each vertex

    Vertices _vertices;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _nodes;

    std::vector<Eigen::Matrix3d> _Rs; // rotation matrics
    std::vector<Eigen::Vector3d> _ts; // translations

    std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> _kd_tree;
};

#endif // EMBEDDED_DEFORMATION_H_LF
