#ifndef CYLINDER_GENERATOR_H_LF
#define CYLINDER_GENERATOR_H_LF
#include "generator_base.h"

namespace mlayer{

/**
 * @brief A class for generating cylinder vertices
 *     ___     
 *    |   |    
 *    |   |    length
 *    |_._|    radius
 *      origin
 */
class CylinderGenerator : public GeneratorBase
{
public:
    /**
     * @brief Constructor of class CylinderGenerator
     * @param length     The length of the cylinder
     * @param radius     The radius of the cylinder
     * @param pose       The pose of the cylinder
     */
    CylinderGenerator(float length, float radius, const glm::mat4& pose);


    /**
     * @brief Constructor of class CylinderGenerator
     * @param length     The length of the cylinder
     * @param radius     The radius of the cylinder
     * @param origin     The origin of the cylinder
     */
    CylinderGenerator(float length, float radius, const glm::vec3& origin);


    /**
     * @brief Return cylinder vertices without end circle.
     * @return
     */
    const Vertices& verticesWithoutEndFace() const;

private:
    void createVertices(float length, float radius, const glm::mat4& pose);

private:
    Vertices _vertices_without_end_face;
};

} // namespace::mlayer
#endif // CYLINDERGENERATOR_H
