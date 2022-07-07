#ifndef CIRCLE_GENERATOR_H_LF
#define CIRCLE_GENERATOR_H_LF
#include <vector>
#include "generator_base.h"

class CircleGenerator : public GeneratorBase
{
public:
    CircleGenerator();
    CircleGenerator(int points_num, const glm::vec3& origin, float radius);
    CircleGenerator(int points_num, const glm::mat4& pose, float radius);

    const Positions& positions() const;

private:
    void createVertices(const glm::vec3& origin, int points_num);

    Positions   _positions;
};

#endif // CIRCLEGENERATOR_H
