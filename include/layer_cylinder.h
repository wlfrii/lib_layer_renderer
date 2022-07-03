#ifndef LAYER_CYLINDER_H_LF
#define LAYER_CYLINDER_H_LF
#include "layer_model.h"

class LayerGripper;

struct CylinderProperty
{
    float     radius;
    float     len;
    glm::vec3 origin;
};


class LayerCylinder : public LayerModel
{
    friend LayerGripper;
public:
    LayerCylinder(uint16_t width, uint16_t height, LayerType type,
                  LayerRenderMode mode, glm::vec3 color,
                  const CylinderProperty& prop);
    ~LayerCylinder();
};

#endif // LAYER_CYLINDER_H_LF
