#ifndef CYLINDER_GENERATOR_H_LF
#define CYLINDER_GENERATOR_H_LF
#include "generator_base.h"


class CylinderGenerator : public GeneratorBase
{
public:
    CylinderGenerator();
    CylinderGenerator(int points_num, const glm::vec3& origin, float length,
                      float radius);
    CylinderGenerator(int points_num, float length, float radius);

    const Vertices& resultWithoutEndFace() const;

private:
    void createVertices(int points_num, const glm::vec3& origin, float length,
                        float radius);

private:
    Vertices _vertices_without_end_face;
};

#endif // CYLINDERGENERATOR_H
