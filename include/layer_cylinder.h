#ifndef LAYER_CYLINDER_H_LF
#define LAYER_CYLINDER_H_LF
#include "layer_model.h"


struct LayerCylinderProperty
{
    LayerCylinderProperty(const glm::vec3& origin = glm::vec3(0.f, 0.f, 0.f),
                          float length = 0, float radius = 0)
        : origin(origin), length(length), radius(radius) {}

    LayerCylinderProperty(float length, float radius)
        : origin(glm::vec3(0.f, 0.f, 0.f)), length(length), radius(radius) {}

    glm::vec3 origin;
    float     length;
    float     radius;
};


class LayerCylinder : public LayerModel
{
public:
    LayerCylinder(uint16_t width, uint16_t height, LayerRenderMode mode,
                  glm::vec3 color, const LayerCylinderProperty& prop);
    ~LayerCylinder();

    void setProperty(const LayerCylinderProperty& prop);
};

#endif // LAYER_CYLINDER_H_LF
