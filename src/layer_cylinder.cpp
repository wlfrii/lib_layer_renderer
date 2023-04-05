#include "../include/lib_layer_renderer/layer_cylinder.h"
#include "./shapes/cylinder_generator.h"
#include <gl_util.h>

namespace mlayer{

LayerCylinder::LayerCylinder(float length, float radius, const glm::vec3 &color,
                             const glm::mat4& pose)
    : LayerModel(LAYER_CYLINDER, color)
    , _pose(pose)
    , _length(length)
    , _radius(radius)
{
    setProperty(length, radius, pose);
}


LayerCylinder::~LayerCylinder()
{

}


void LayerCylinder::setProperty(float length, float radius, const glm::mat4 &pose)
{
    _pose = pose;
    _length = length;
    _radius = radius;

    CylinderGenerator cg(length, radius, pose);
    auto data = cg.vertices();

    _vavbebo->bind(&data[0].position.x, data.size() * sizeof(Vertex));
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(1);
    _vert_num = data.size();
}


void LayerCylinder::setProperty(float length, float radius)
{
    _radius = radius;
    setProperty(length, radius, _pose);
}


void LayerCylinder::setProperty(float length)
{
    setProperty(length, _radius, _pose);
}


void LayerCylinder::setScaleFactors(float xscale, float yscale, float zscale)
{
    CylinderGenerator cg(_length, _radius, _pose);
    auto data = cg.vertices();
    for(auto& vert : data) {
        vert.position.x *= xscale;
        vert.position.y *= yscale;
        vert.position.z *= zscale;
    }

    _vavbebo->bind(&data[0].position.x, data.size() * sizeof(Vertex));
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(1);
    _vert_num = data.size();
}

} // namespace::mlayer
