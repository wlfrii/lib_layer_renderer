#ifndef CIRCLE_GENERATOR_H_LF
#define CIRCLE_GENERATOR_H_LF
#include "generator_base.h"

namespace mlayer{

/**
 * @brief A class for generating circle vertices
 */
class CircleGenerator : public GeneratorBase
{
public:
    /**
     * @brief Constructor of class CircleGenerator
     * @param radius     The radius of the circle
     * @param pose       The pose of the circle
     */
    CircleGenerator(float radius, const glm::mat4& pose);


    /**
     * @brief Constructor of class CircleGenerator
     * @param radius     The radius of the circle
     * @param points_num The number of points in the circle
     * @param pose       The pose of the circle
     */
    CircleGenerator(float radius, int points_num, const glm::mat4& pose);


    /**
     * @brief Return the vertex positions of the circle
     * @return
     */
    const VertexPositions& vertexPositions() const;


    /**
     * @brief Return the Vertices with flipped normal since circle has two faces
     * @return
     */
    Vertices normalFlippedVertices() const;


private:
    /* Create the circle vertices */
    void createVertices(float radius, int points_num, const glm::mat4 &pose);

    VertexPositions   _vert_positions;  //!< vertex positions of the circle
};

} // namespace::mlayer
#endif // CIRCLEGENERATOR_H
