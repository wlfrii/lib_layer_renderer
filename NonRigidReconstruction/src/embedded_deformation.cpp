#include "embedded_deformation.h"
#include <vector>

namespace ED {
void createNodes(const Points3D& vertices, Points3D& nodes, float nodes_density);
}


EmbeddedDeformation::EmbeddedDeformation(Points3D init_vertices,
                                         float point_cloud_density,
                                         float node_density)
{


}


EmbeddedDeformation::~EmbeddedDeformation()
{

}


/* ------------------------------------------------------------------- */
/*                             namespace ED                            */
/* ------------------------------------------------------------------- */
namespace ED {
struct PointWithOccurrences
{
    PointWithOccurrences() : p({0, 0, 0}), n(0) {}
    Eigen::Vector3f p;  // The position of the point
    size_t          n;  // The number of occurrences
};

void createNodes(const Points3D& vertices, Points3D& nodes, float nodes_density)
{
    Eigen::Vector3f max_xyzf = vertices.colwise().maxCoeff();
    Eigen::Vector3f min_xyzf = vertices.colwise().minCoeff();

    size_t x_node_num = floor((max_xyzf[0] - min_xyzf[0]) / nodes_density);
    size_t y_node_num = floor((max_xyzf[1] - min_xyzf[1]) / nodes_density);
    size_t z_node_num = floor((max_xyzf[2] - min_xyzf[2]) / nodes_density);

    // Group vertices into voxels
    std::vector<std::vector<std::vector<PointWithOccurrences>>> voxels(
                x_node_num, std::vector<std::vector<PointWithOccurrences>>(
                    y_node_num, std::vector<PointWithOccurrences>(
                        z_node_num, PointWithOccurrences())));
    for(int i = 0; i < vertices.rows(); i++) {
        int x_idx = floor(vertices(i, 0) - min_xyzf[0] / nodes_density);
        int y_idx = floor(vertices(i, 1) - min_xyzf[1] / nodes_density);
        int z_idx = floor(vertices(i, 2) - min_xyzf[2] / nodes_density);

        voxels[x_idx][y_idx][z_idx].p += vertices.row(i);
        voxels[x_idx][y_idx][z_idx].n++;
    }

    // Filter out the valid voxels
    std::vector<Eigen::Vector3f> valid_voxels;
    for(size_t x_idx = 0; x_idx < x_node_num; x_idx++) {
        for(size_t y_idx = 0; y_idx < y_node_num; y_idx++) {
            for(size_t z_idx = 0; z_idx < z_node_num; z_idx++) {
                PointWithOccurrences& pt = voxels[x_idx][y_idx][z_idx];
                if(pt.n > 0) {
                    valid_voxels.push_back(pt.p / pt.n);
                }
            }
        }
    }

    nodes.resize(valid_voxels.size(), 3);
    for(size_t i = 0; i < valid_voxels.size(); i++) {
        nodes.row(i) = valid_voxels[i];
    }
}

} // ED
