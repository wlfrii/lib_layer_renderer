#ifndef CYLINDER_GENERATOR_H_LF
#define CYLINDER_GENERATOR_H_LF
#include "generator_base.h"


/**
 * @brief A class for generating circle vertices
 */
class CylinderGenerator : public GeneratorBase
{
public:
    /**
     * @brief Default constructor of class CylinderGenerator
     */
    CylinderGenerator();


    /**
     * @brief Constructor of class CylinderGenerator
     * @param points_num The number of points in each end circle
     * @param origin     The origin of the cylinder
     * @param length     The length of the cylinder
     * @param radius     The radius of the cylinder
     */
    CylinderGenerator(int points_num, const glm::vec3& origin, float length,
                      float radius);


    /**
     * @brief Constructor of class CylinderGenerator
     * @param points_num The number of points in the end circle
     * @param length     The length of the cylinder
     * @param radius     The radius of the cylinder
     */
    CylinderGenerator(int points_num, float length, float radius);


    /**
     * @brief Return cylinder vertices without end circle.
     * @return
     */
    const Vertices& resultWithoutEndFace() const;

private:
    void createVertices(int points_num, const glm::vec3& origin, float length,
                        float radius);

private:
    Vertices _vertices_without_end_face;
};

#endif // CYLINDERGENERATOR_H
