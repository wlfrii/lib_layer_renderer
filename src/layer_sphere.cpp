#include "../include/lib_layer_renderer/layer_sphere.h"
#include "./shapes/sphere_generator.h"
#include <gl_util.h>

namespace mlayer{

LayerSphere::LayerSphere(float radius, const glm::vec3 &color,
                     const glm::mat4& pose)
    : LayerModel(LAYER_CONE, color)
    , _pose(pose)
    , _radius(radius)
{
    setProperty(_radius, _pose);
}


LayerSphere::~LayerSphere()
{

}


void LayerSphere::setProperty(float radius, const glm::mat4& pose)
{
    SphereGenerator sg(radius, pose);
    auto data = sg.vertices();

    _vavbebo->bind(&data[0].position.x, data.size() * sizeof(Vertex));
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(1);
    _vert_num = data.size();
}


void LayerSphere::setScaleFactors(float xscale, float yscale, float zscale)
{
    SphereGenerator sg(_radius, _pose);
    auto data = sg.vertices();

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
