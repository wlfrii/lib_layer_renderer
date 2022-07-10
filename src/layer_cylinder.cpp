#include "../include/layer_cylinder.h"
#include "./shapes/cylinder_generator.h"
#include <gl_util.h>
#include <global.h>

LayerCylinder::LayerCylinder(uint16_t width, uint16_t height,
                             LayerRenderMode mode, glm::vec3 color,
                             const LayerCylinderProperty &prop)
    : LayerModel(width, height, mode, color)
{
    _vavbebo = new gl_util::VAVBEBO();
    setProperty(prop);
}


LayerCylinder::~LayerCylinder()
{

}


void LayerCylinder::setProperty(const LayerCylinderProperty &prop)
{
    int num = (int)prop.radius*3.1415926*2 / 0.5f;
    CylinderGenerator cg(num, prop.origin, prop.length, prop.radius);
    auto data = cg.result();

    _vavbebo->bind(&data[0].position.x, data.size() * sizeof(Vertex));
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(1);
    _vert_num = data.size();
}

