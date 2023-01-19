#include "../include/layer_circle.h"
#include "./shapes/circle_generator.h"
#include <gl_util.h>
#include <global.h>


LayerCircle::LayerCircle(float radius, const glm::vec3 &color,
                         const glm::mat4& pose)
    : LayerModel(LAYER_CIRCLE, color)
    , _pose(pose)
    , _radius(radius)
{
    setProperty(pose, radius);
}


LayerCircle::~LayerCircle()
{

}


void LayerCircle::setProperty(const glm::mat4 &pose, float radius)
{
    int num = (int)radius*3.1415926*2 / 0.7f;
    CircleGenerator cg(num, radius, pose);
    auto data = cg.result();

    _vavbebo->bind(&data[0].position.x, data.size() * sizeof(Vertex));
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(1);
    _vert_num = data.size();
}


void LayerCircle::setProperty(float radius)
{
    _radius = radius;
    setProperty(_pose, radius);
}

