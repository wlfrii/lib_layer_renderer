#include "cone_generator.h"
#include "circle_generator.h"
#include "generator_util.h"


ConeGenerator::ConeGenerator(float height, float radius, const glm::mat4 &pose)
{
    // Create positions
    CircleGenerator cir(radius, pose);
    const auto& pos = cir.vertexPositions();

    glm::vec4 p = pose * glm::vec4(0, 0, height, 1);

    // Create vertices
    int points_num = pos.size();
    _vertices.resize(points_num * 3);
    for(int i = 0; i < points_num; i++){
        const glm::vec4& p1 = pos[i];
        const glm::vec4& p2 = pos[(i + 1) % points_num];
        glm::vec4 n = getNormal(p1, p2, p);

        _vertices[i*3 + 0] = {p1, n};
        _vertices[i*3 + 1] = {p2, n};
        _vertices[i*3 + 2] = {p, n};
    }

    _vertices.insert(_vertices.end(), cir.vertices().begin(), cir.vertices().end());
}
