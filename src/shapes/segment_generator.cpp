#include "segment_generator.h"
#include "circle_generator.h"
#include "generator_util.h"
#include "../include/layer_util.h"
#include <lib_math/lib_math.h>

SegmentGenerator::SegmentGenerator()
{

}

SegmentGenerator::SegmentGenerator(int points_num, float length, float len_gap,
                                   float theta, float delta, float radius)
{
    std::vector<float> L;
    mmath::linspace(0, len_gap, length, L);

    std::vector<CircleGenerator> circles(L.size());
    circles[0] = CircleGenerator(points_num, glm::vec3(0.f,0.f,0.f), radius);

    mmath::Pose endpose;
    for(size_t i = 1; i < L.size(); i++){
        float t_len = L[i];
        float t_theta = t_len / length * theta;
        endpose = mmath::continuum::calcSingleSegmentPose(t_len, t_theta, delta);

        circles[i] = CircleGenerator(points_num, cvt2GlmMat4(endpose), radius);
    }

    _vertices_spacers.clear();
    for(auto& cc : circles){
        _vertices_spacers.insert(_vertices_spacers.end(),
                                 cc.result().begin(), cc.result().end());
    }
    //printf("_vertices_spacers.size:%ld\n", _vertices_spacers.size());

    _vertices.clear();
    for(size_t i = 0; i < L.size() - 1; i++){
        auto& pos1 = circles[i].positions();
        auto& pos2 = circles[i + 1].positions();

        Vertices vertices(points_num * 6);
        for(int i = 0; i < points_num; i++){
            size_t idx0 = i;
            size_t idx1 = (i + 1) % points_num;

            const glm::vec4& a = pos1[idx0];
            const glm::vec4& b = pos1[idx1];
            const glm::vec4& c = pos2[idx0];
            const glm::vec4& d = pos2[idx1];

            glm::vec4 n = getNormal(a, d, c);
            vertices[i*6 + 0] = {a, n};
            vertices[i*6 + 1] = {d, n};
            vertices[i*6 + 2] = {c, n};

            n = getNormal(a, b, d);
            vertices[i*6 + 3] = {a, n};
            vertices[i*6 + 4] = {b, n};
            vertices[i*6 + 5] = {d, n};
        }
        _vertices.insert(_vertices.end(), vertices.begin(), vertices.end());
    }
}


const Vertices &SegmentGenerator::resultSpacers() const
{
    return _vertices_spacers;
}


