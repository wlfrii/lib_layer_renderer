#include "cone_generator.h"
#include "circle_generator.h"
#include "generator_util.h"


ConeGenerator::ConeGenerator(int points_num, float height, float radius)
{
    assert(points_num >= 3);

    glm::mat4 pose(1.0f);
    createVertices(points_num, pose, height, radius);
}


ConeGenerator::ConeGenerator(int points_num, const glm::mat4 &pose,
                             float height, float radius)
{
    assert(points_num >= 3);

    createVertices(points_num, pose, height, radius);
}


void ConeGenerator::createVertices(int points_num, const glm::mat4 &pose,
                                   float height, float radius)
{
    // Create positions
    CircleGenerator circle(points_num, radius, pose);
    const auto& pos = circle.vertexPositions();

    glm::vec4 p = pose * glm::vec4(0, 0, height, 1);

    // Create vertices
    _vertices.resize(points_num * 3);
    for(int i = 0; i < points_num; i++){
        const glm::vec4& p1 = pos[i];
        const glm::vec4& p2 = pos[(i + 1) % points_num];
        glm::vec4 n = getNormal(p1, p, p2);

        _vertices[i*3 + 0] = {p1, n};
        _vertices[i*3 + 1] = {p2, n};
        _vertices[i*3 + 2] = {p, n};
    }
}
