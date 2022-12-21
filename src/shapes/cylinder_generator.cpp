#include "cylinder_generator.h"
#include "circle_generator.h"
#include "generator_util.h"


CylinderGenerator::CylinderGenerator(int points_num, const glm::vec3& origin,
                                     float length, float radius)
{
    createVertices(points_num, origin, length, radius);
}


CylinderGenerator::CylinderGenerator(int points_num, float length, float radius)
{
    createVertices(points_num, glm::vec3(0, 0, 0), length, radius);
}


const Vertices &CylinderGenerator::resultWithoutEndFace() const
{
    return _vertices_without_end_face;
}


void CylinderGenerator::createVertices(int points_num, const glm::vec3 &origin,
                                       float length, float radius)
{
    glm::vec3 o1 = origin;
    CircleGenerator c1(points_num, o1, radius);
    glm::vec3 o2 = origin;
    o2.z += length;
    CircleGenerator c2(points_num, o2, radius);

    const auto& pos1 = c1.vertexPositions();
    const auto& pos2 = c2.vertexPositions();
    const size_t num = pos1.size();

    _vertices_without_end_face.resize(num * 6);
    for(size_t i = 0; i < num; i++){
        size_t idx0 = i;
        size_t idx1 = (i + 1) % num;

        const glm::vec4& a = pos1[idx0];
        const glm::vec4& b = pos1[idx1];
        const glm::vec4& c = pos2[idx0];
        const glm::vec4& d = pos2[idx1];

        glm::vec4 n = getNormal(a, d, c);
        _vertices_without_end_face[i*6 + 0] = {a, n};
        _vertices_without_end_face[i*6 + 1] = {d, n};
        _vertices_without_end_face[i*6 + 2] = {c, n};

        n = getNormal(a, b, d);
        _vertices_without_end_face[i*6 + 3] = {a, n};
        _vertices_without_end_face[i*6 + 4] = {b, n};
        _vertices_without_end_face[i*6 + 5] = {d, n};
    }

    _vertices = _vertices_without_end_face;
    _vertices.insert(_vertices.end(), c1.result().begin(), c1.result().end());
    _vertices.insert(_vertices.end(), c2.result().begin(), c2.result().end());
}
