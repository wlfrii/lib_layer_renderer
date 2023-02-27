#ifndef EMBEDDED_DEFORMATION_H_LF
#define EMBEDDED_DEFORMATION_H_LF
#include <Eigen/Dense>


using Points3D = Eigen::Matrix<float, Eigen::Dynamic, 3>;


struct Coord3D {
    float x;
    float y;
    float z;
};

struct Color {
    float r;
    float g;
    float b;
};

struct Vertex {
    Coord3D coord;
    float weight;
    Color color;
};


/**
 * @brief The EmbeddedDeformation class is designed for parameterize the norigid
 * deformation field.
 *
 */
class EmbeddedDeformation
{
public:
    EmbeddedDeformation(Points3D init_vertices, float point_cloud_density,
                        float node_density);
    ~EmbeddedDeformation();

private:



    Points3D _vertices;
    Points3D _ED_nodes;
};

#endif // EMBEDDED_DEFORMATION_H_LF
