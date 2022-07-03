#ifndef CYLINDER_GENERATOR_H_LF
#define CYLINDER_GENERATOR_H_LF
#include <vector>
#include <glm/glm.hpp>
#include "../layer_define.h"


class CylinderGenerator
{
public:
    CylinderGenerator();
    CylinderGenerator(int points_num, float radius, float len,
                      const glm::vec3& origin = glm::vec3(0.f, 0.f, 0.f));

    const Vertices& result() const;
    const Vertices& resultWithoutEndFace() const;

private:
    Vertices _vertices_without_end_face;
    Vertices _vertices_with_end_face;
};

#endif // CYLINDERGENERATOR_H
