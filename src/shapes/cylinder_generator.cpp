#include "cylinder_generator.h"
#include "circle_generator.h"
#include "util_generator.h"


CylinderGenerator::CylinderGenerator()
{

}


CylinderGenerator::CylinderGenerator(int points_num, float radius, float len,
                                     const glm::vec3& origin)
{
    glm::vec3 o1 = origin;  o1.z -= len/ 2.0;
    CircleGenerator c1(points_num, radius, o1);
    glm::vec3 o2 = origin;  o2.z += len/ 2.0;
    CircleGenerator c2(points_num, radius, o2);

    const auto& pos1 = c1.positions();
    const auto& pos2 = c2.positions();
    const size_t n = pos1.size();

    _vertices_without_end_face.resize(n * 6);
    for(size_t i = 0; i < n; i++){
        size_t idx0 = i;
        size_t idx1 = (i + 1) % n;

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

    _vertices_with_end_face = _vertices_without_end_face;
    _vertices_with_end_face.insert(_vertices_with_end_face.end(),
                                   c1.result().begin(), c1.result().end());
    _vertices_with_end_face.insert(_vertices_with_end_face.end(),
                                   c2.result().begin(), c2.result().end());
}


const Vertices &CylinderGenerator::result() const
{
    return _vertices_with_end_face;
}


const Vertices &CylinderGenerator::resultWithoutEndFace() const
{
    return _vertices_without_end_face;
}
