#include "embedded_deformation.h"
#include <vector>
#include "util.h"


EmbeddedDeformation::EmbeddedDeformation(float node_density, int node_connectivity)
    : _node_density(node_density)
    , _node_connectivity(node_connectivity)
    , _vertices(new pcl::PointCloud<pcl::PointXYZRGB>)
    , _ed_nodes(new pcl::PointCloud<pcl::PointXYZ>)
    , _kd_tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>)
{
}


EmbeddedDeformation::~EmbeddedDeformation()
{

}


void EmbeddedDeformation::addVertices(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertices)
{
    _vertices = vertices;
    util::voxelDownSampling(vertices, _node_density, _ed_nodes);

    _Rs = std::vector<Eigen::Matrix3f>(_ed_nodes->size(),
                                       Eigen::Matrix3f::Identity());
    _ts = std::vector<Eigen::Vector3f>(_ed_nodes->size(),
                                       Eigen::Vector3f::Zero());
}


void EmbeddedDeformation::addVertices(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr vertices,
        const std::vector<
            std::pair<pcl::PointXYZRGB, pcl::PointXYZRGB>>& correspondences)
{
    int K = _node_connectivity + 1;

    // Find the neighor nodes of each deformed vertices
    std::vector<std::vector<int>> src_vertices_neighbor_nodes(
                correspondences.size(), std::vector<int>(K));

    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_N;
    kd_tree_N.setInputCloud(_ed_nodes);

    for(size_t i = 0; i < correspondences.size(); i++) {
        const pcl::PointXYZRGB& src_pt = correspondences[i].first;

        std::vector<int> indices(K);
        std::vector<float> squared_distances(K);
        src_vertices_neighbor_nodes[i] = kd_tree_N.nearestKSearch(
                    src_pt, K, indices, squared_distances);

    }

}


void EmbeddedDeformation::updateKdTreeData()
{
    _kd_tree->setInputCloud(_vertices);
}


