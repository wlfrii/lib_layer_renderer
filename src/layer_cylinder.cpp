#include "../include/layer_cylinder.h"
#include "./shapes/cylinder_generator.h"
#include <gl_util.h>
#include <global.h>

LayerCylinder::LayerCylinder(const glm::vec3 &origin, float length,
                             float radius, const glm::vec3 &color)
    : LayerModel(LAYER_CYLINDER, color)
    , _origin(origin)
    , _radius(radius)
{
    setProperty(origin, length, radius);
}


LayerCylinder::LayerCylinder(float length, float radius, const glm::vec3 &color)
    : LayerModel(LAYER_CYLINDER, color)
    , _origin(glm::vec3(0.f, 0.f, 0.f))
    , _radius(radius)
{
    setProperty(length, radius);
}


LayerCylinder::~LayerCylinder()
{

}


void LayerCylinder::setProperty(const glm::vec3 &origin, float length, float radius)
{
    _origin = origin;
    _radius = radius;

    int num = (int)radius*3.1415926*2 / 0.7f;
    CylinderGenerator cg(num, origin, length, radius);
    auto data = cg.result();

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
    setProperty(_origin, length, radius);
}


void LayerCylinder::setProperty(float length)
{
    setProperty(_origin, length, _radius);
}
