#include "../include/layer_cone.h"
#include "./shapes/cone_generator.h"
#include <gl_util.h>

namespace mlayer{

LayerCone::LayerCone(float height, float radius, const glm::vec3 &color,
                     const glm::mat4& pose)
    : LayerModel(LAYER_CONE, color)
    , _pose(pose)
    , _radius(radius)
    , _height(height)
{
    setProperty(_height, _radius, _pose);
}


LayerCone::~LayerCone()
{

}


void LayerCone::setProperty(float height, float radius, const glm::mat4& pose)
{
    ConeGenerator cg(height, radius, pose);
    auto data = cg.vertices();

    _vavbebo->bind(&data[0].position.x, data.size() * sizeof(Vertex));
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(1);
    _vert_num = data.size();
}


void LayerCone::setHeight(float height)
{
    _height = height;
    setProperty(_height, _radius, _pose);
}

} // namespace::mlayer