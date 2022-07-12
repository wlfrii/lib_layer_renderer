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

    int num = (int)radius*3.1415926*2 / 0.7f;
    float len_gap = 2.0f;

    Vertices data;
    if(abs(theta) < 1e-4){
        CylinderGenerator cg(num, length, radius);
        data = cg.result();
    }
    else{
        SegmentGenerator sg(num, length, len_gap, theta, delta, radius);
        data = sg.result();
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
