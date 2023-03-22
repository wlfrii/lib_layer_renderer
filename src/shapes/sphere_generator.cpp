#include "sphere_generator.h"
#include "circle_generator.h"
#include "generator_util.h"

namespace mlayer{

namespace {
const uint16_t MIN_POINTS_NUM = 8;
const uint16_t MIN_CIRCLES_NUM = 8;
const float SPHERE_PRECISION = 0.25;
}

SphereGenerator::SphereGenerator(float radius, const glm::mat4 &pose)
{
    // The number of points of each circle
    int points_num = (int)radius*3.1415926*2 / SPHERE_PRECISION;
    if (points_num < MIN_POINTS_NUM){
        points_num = MIN_POINTS_NUM;
    }

    int circles_num = (int)radius*2 / SPHERE_PRECISION;
    if (circles_num < MIN_CIRCLES_NUM){
        circles_num = MIN_CIRCLES_NUM;
    }
    float dgap = radius*2.f / circles_num;

    // Create positions
    glm::vec4 p_top = pose * glm::vec4(0, 0, radius, 1);
    glm::vec4 p_but = pose * glm::vec4(0, 0, -radius, 1);

    glm::mat4 center_pose = pose;
    std::vector<CircleGenerator*> circles(circles_num - 1);
    for(int i = 1; i < circles_num; i++) {
        float h = radius - i*dgap;
        float r = sqrtf(radius * radius - h * h);

        center_pose[3][2] = h;
        circles[i - 1] = new CircleGenerator(r, points_num, center_pose);
    }

    // Create vertices
    _vertices.clear();
    Vertices vertices(points_num * 3);
    const auto& pos_top = circles[0]->vertexPositions();
    for(int i = 0; i < points_num; i++){
        const glm::vec4& p1 = pos_top[i];
        const glm::vec4& p2 = pos_top[(i + 1) % points_num];
        glm::vec4 n = getNormal(p1, p2, p_top);

        vertices[i*3 + 0] = {p1, n};
        vertices[i*3 + 1] = {p2, n};
        vertices[i*3 + 2] = {p_top, n};
    }
    _vertices.insert(_vertices.end(), vertices.begin(), vertices.end());
    const auto& pos_but = circles.back()->vertexPositions();
    for(int i = 0; i < points_num; i++){
        const glm::vec4& p1 = pos_but[i];
        const glm::vec4& p2 = pos_but[(i + 1) % points_num];
        glm::vec4 n = getNormal(p1, p_but, p2);

        vertices[i*3 + 0] = {p1, n};
        vertices[i*3 + 1] = {p_but, n};
        vertices[i*3 + 2] = {p2, n};
    }
    _vertices.insert(_vertices.end(), vertices.begin(), vertices.end());

    for(size_t i = 0; i < circles.size() - 1; i++){
        auto& pos1 = circles[i]->vertexPositions();
        auto& pos2 = circles[i + 1]->vertexPositions();

        Vertices vertices(points_num * 6);
        for(int i = 0; i < points_num; i++){
            size_t idx0 = i;
            size_t idx1 = (i + 1) % points_num;

            const glm::vec4& a = pos1[idx0];
            const glm::vec4& b = pos1[idx1];
            const glm::vec4& c = pos2[idx0];
            const glm::vec4& d = pos2[idx1];

            glm::vec4 n = getNormal(a, c, d);
            vertices[i*6 + 0] = {a, n};
            vertices[i*6 + 2] = {c, n};
            vertices[i*6 + 1] = {d, n};

            n = getNormal(a, d, b);
            vertices[i*6 + 3] = {a, n};
            vertices[i*6 + 5] = {d, n};
            vertices[i*6 + 4] = {b, n};
        }
        _vertices.insert(_vertices.end(), vertices.begin(), vertices.end());
    }

    for(auto& cc : circles){
        delete cc;
        cc = nullptr;
    }
}

} // namespace::mlayer
