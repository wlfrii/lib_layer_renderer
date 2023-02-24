#include "cylinder_generator.h"
#include "circle_generator.h"
#include "generator_util.h"

namespace mlayer{

CylinderGenerator::CylinderGenerator(float length, float radius,
                                     const glm::mat4& pose)
{
    createVertices(length, radius, pose);
}


CylinderGenerator::CylinderGenerator(float length, float radius,
                                     const glm::vec3& origin)
{
    glm::mat4 pose(1.f);
    pose[3][0] = origin[0];
    pose[3][1] = origin[1];
    pose[3][2] = origin[2];
    createVertices(length, radius, pose);
}


const Vertices &CylinderGenerator::verticesWithoutEndFace() const
{
    return _vertices_without_end_face;
}


void CylinderGenerator::createVertices(float length, float radius,
                                       const glm::mat4& pose)
{
    CircleGenerator c1(radius, pose);
    glm::mat4 end_pose = pose;
    end_pose[3][0] += length * end_pose[2][0];
    end_pose[3][1] += length * end_pose[2][1];
    end_pose[3][2] += length * end_pose[2][2];
    CircleGenerator c2(radius, end_pose);

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
    _vertices.insert(_vertices.end(), c1.vertices().begin(), c1.vertices().end());
    _vertices.insert(_vertices.end(), c2.vertices().begin(), c2.vertices().end());
}

} // namespace::mlayer