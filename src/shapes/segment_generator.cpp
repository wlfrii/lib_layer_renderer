#include "segment_generator.h"
#include "circle_generator.h"
#include "generator_util.h"
#include "../include/layer_util.h"
#include <lib_math/lib_math.h>

SegmentGenerator::SegmentGenerator()
{

}

SegmentGenerator::SegmentGenerator(float length, float len_gap,
                                   float theta, float delta, float radius)
{
    std::vector<float> L;
    mmath::linspace(0, len_gap, length, L);

    std::vector<CircleGenerator*> circles(L.size());
    circles[0] = new CircleGenerator(radius, glm::mat4(1.f));

    mmath::Pose endpose;
    for(size_t i = 1; i < L.size(); i++){
        float t_len = L[i];
        float t_theta = t_len / length * theta;
        endpose = mmath::continuum::calcSingleSegmentPose(t_len, t_theta, delta);

        circles[i] = new CircleGenerator(radius, cvt2GlmMat4(endpose));
    }

    _vertices_spacers.clear();
    for(auto& cc : circles){
        _vertices_spacers.insert(_vertices_spacers.end(),
                                 cc->vertices().begin(), cc->vertices().end());
    }
    //printf("_vertices_spacers.size:%ld\n", _vertices_spacers.size());
    int points_num = circles[0]->vertexPositions().size();

    _vertices.clear();
    for(size_t i = 0; i < L.size() - 1; i++){
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
    auto& c1 = circles[0];
    auto& c2 = circles[circles.size() - 1];
    _vertices.insert(_vertices.end(), c1->vertices().begin(), c1->vertices().end());
    _vertices.insert(_vertices.end(), c2->vertices().begin(), c2->vertices().end());

    for(auto& cc : circles){
        delete cc;
        cc = nullptr;
    }
}


const Vertices &SegmentGenerator::spacerVertices() const
{
    return _vertices_spacers;
}


