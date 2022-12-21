#include "circle_generator.h"
#include "generator_util.h"


CircleGenerator::CircleGenerator(int points_num, const glm::vec3& origin,
                                 float radius)
{
    assert(points_num >= 3);

    // Build circle points
    _vert_positions.resize(points_num);
    double angle = 2.0*M_PI / points_num;
    for(int i = 0; i < points_num; i++){
        float delta = angle * i;
        float x = origin.x + radius * glm::cos(delta);
        float y = origin.y + radius * glm::sin(delta);
        float z = origin.z;
        _vert_positions[i] = glm::vec4(x, y, z, 1);
    }

    // Build circle vertices with normal
    createVertices(origin, points_num);
}


CircleGenerator::CircleGenerator(int points_num, const glm::mat4 &pose, float radius)
{
    assert(points_num >= 3);

    // Build circle points
    _vert_positions.resize(points_num);
    double angle = 2.0*M_PI / points_num;
    for(int i = 0; i < points_num; i++){
        float delta = angle * i;
        float x = radius * glm::cos(delta);
        float y = radius * glm::sin(delta);
        float z = 0;
        _vert_positions[i] = pose * glm::vec4(x, y, z, 1);
    }

    // Build circle vertices with normal
    createVertices(glm::vec3(pose[3][0], pose[3][1], pose[3][2]), points_num);
}


const VertexPositions& CircleGenerator::vertexPositions() const
{
    return _vert_positions;
}


void CircleGenerator::createVertices(const glm::vec3& origin, int points_num)
{
    _vertices.resize(points_num * 3);
    glm::vec4 p = glm::vec4(origin, 1.f);
    for(int i = 0; i < points_num; i++){
        glm::vec4 p1 = _vert_positions[i];
        glm::vec4 p2 = _vert_positions[(i + 1) % points_num];
        glm::vec4 n = getNormal(p1, p, p2);

        _vertices.push_back({p1, n});
        _vertices.push_back({p2, n});
        _vertices.push_back({p, n});
    }
}
