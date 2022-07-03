#ifndef CIRCLE_GENERATOR_H_LF
#define CIRCLE_GENERATOR_H_LF
#include <vector>
#include <glm/glm.hpp>
#include "../layer_define.h"

class CircleGenerator
{
public:
    CircleGenerator();
    CircleGenerator(int points_num, float radius, const glm::vec3& origin = glm::vec3(0.f, 0.f, 0.f));

    const Vertices& result() const;
    const Positions& positions() const;

//    void negZ();

private:
    int         _points_num;    // must be >= 3
    float       _radius;
    glm::vec3   _origin;

    Vertices    _vertices;
    Positions   _positions;
};

#endif // CIRCLEGENERATOR_H
