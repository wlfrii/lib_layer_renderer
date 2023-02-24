#include "../include/layer_segment.h"
#include "../include/layer_util.h"
#include "shapes/segment_generator.h"
#include "shapes/cylinder_generator.h"
#include <lib_math/lib_math.h>
#include <gl_util.h>


LayerSegment::LayerSegment(float length, float theta, float delta, float radius,
                           const glm::vec3 &color)
    : LayerModel(LAYER_SEGMENT, color)
    , _radius(radius)
{
    setProperty(length, theta, delta, radius);
}


LayerSegment::~LayerSegment()
{

}


void LayerSegment::setProperty(float length, float theta, float delta,
                               float radius)
{
    _radius = radius;

    Vertices data;
    if(abs(theta) < 1e-4){
        CylinderGenerator cg(length, radius, glm::mat4(1.f));
        data = cg.vertices();
    }
    else{
        float len_gap = std::fmaxf(length / 25.f / theta, 1.f);
        SegmentGenerator sg(length, len_gap, theta, delta, radius);
        data = sg.vertices();
        //auto data = sg.resultSpacers();
    }

    _vavbebo->bind(&data[0].position.x, data.size() * sizeof(Vertex));
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(1);
    _vert_num = data.size();
}


void LayerSegment::setProperty(float length, float theta, float delta)
{
    setProperty(length, theta, delta, _radius);
}


void LayerSegment::setProperty(const mmath::continuum::ConfigSpc &q)
{
    setProperty(q.length, q.theta, q.delta);
}
