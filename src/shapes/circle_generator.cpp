#include "circle_generator.h"
#include "generator_util.h"


CircleGenerator::CircleGenerator(int points_num, float radius,
                                 const glm::mat4& pose)
{
    assert(points_num >= 3);

    // Build circle vertices with normal
    createVertices(points_num, pose, radius);
}


CircleGenerator::CircleGenerator(int points_num, float radius,
                                 const glm::vec3& origin)
{
    assert(points_num >= 3);

    glm::mat4 pose(1.0f);
    pose[3][0] = origin[0];
    pose[3][1] = origin[1];
    pose[3][2] = origin[2];

    // Build circle vertices with normal
    createVertices(points_num, pose, radius);
}




const VertexPositions& CircleGenerator::vertexPositions() const
{
    return _vert_positions;
}


Vertices CircleGenerator::flipVertexNormal() const
{
    Vertices vertices = _vertices;
    for(auto& vert : vertices){
        glm::vec4 n = vert.normal;
        vert.normal = glm::vec4(-n[0], -n[1], -n[2], 1.0f);
    }
    return vertices;
}


void CircleGenerator::createVertices(int points_num, const glm::mat4 &pose,
                                     float radius)
{
    // Create positions
    _vert_positions.resize(points_num);
    double angle = 2.0*M_PI / points_num;
    for(int i = 0; i < points_num; i++){
        float delta = angle * i;
        float x = radius * glm::cos(delta);
        float y = radius * glm::sin(delta);
        float z = 0;
        _vert_positions[i] = pose * glm::vec4(x, y, z, 1);
    }

    // Create vertices
    _vertices.resize(points_num * 3);
    glm::vec4 p = glm::vec4(pose[3][0], pose[3][1], pose[3][2], 1.f);
    for(int i = 0; i < points_num; i++){
        const glm::vec4& p1 = _vert_positions[i];
        const glm::vec4& p2 = _vert_positions[(i + 1) % points_num];
        glm::vec4 n = getNormal(p1, p, p2);

        _vertices[i*3 + 0] = {p1, n};
        _vertices[i*3 + 1] = {p2, n};
        _vertices[i*3 + 2] = {p, n};
    }
}
