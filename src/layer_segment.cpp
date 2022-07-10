#include "../include/layer_segment.h"
#include "../include/layer_util.h"
#include "shapes/segment_generator.h"
#include <lib_math/lib_math.h>
#include <gl_util.h>


LayerSegment::LayerSegment(uint16_t width, uint16_t height, LayerType type,
                           LayerRenderMode mode, glm::vec3 color,
                           const LayerSegmentProperty &prop)
    : LayerModelBase(width, height, type, mode, color)
{
    _vavbebo = new gl_util::VAVBEBO();
    setProperty(prop);
}


LayerSegment::~LayerSegment()
{

}


void LayerSegment::setProperty(const LayerSegmentProperty &prop)
{
    int num = (int)prop.radius*3.1415926*2 / 0.5f;
    float len_gap = 1.f;
    SegmentGenerator sg(num, prop.length, len_gap, prop.theta, prop.delta,
                        prop.radius);
    //auto data = sg.resultSpacers();
    auto data = sg.result();

    _vavbebo->bind(&data[0].position.x, data.size() * sizeof(Vertex));
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(1);
    _vert_num = data.size();
}
