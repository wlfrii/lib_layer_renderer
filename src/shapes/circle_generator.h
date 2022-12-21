#ifndef CIRCLE_GENERATOR_H_LF
#define CIRCLE_GENERATOR_H_LF
#include "generator_base.h"


/**
 * @brief A class for generating circle vertices
 */
class CircleGenerator : public GeneratorBase
{
public:
    /**
     * @brief Constructor of class CircleGenerator
     * @param points_num The number of points in the circle
     * @param origin     The origin of the circle
     * @param radius     The radius of the circle
     */
    CircleGenerator(int points_num, const glm::vec3& origin, float radius);


    /**
     * @brief Constructor of class CircleGenerator
     * @param points_num The number of points in the circle
     * @param pose       The pose of the circle
     * @param radius     The radius of the circle
     */
    CircleGenerator(int points_num, const glm::mat4& pose, float radius);


    /**
     * @brief Return the vertex positions of the circle
     * @return
     */
    const VertexPositions& vertexPositions() const;

private:
    void createVertices(const glm::vec3& origin, int points_num);

    VertexPositions   _vert_positions;  //!< vertex positions of the circle
};

#endif // CIRCLEGENERATOR_H
