#ifndef LAYER_CYLINDER_H_LF
#define LAYER_CYLINDER_H_LF
#include "layer_model.h"


class LayerCylinder : public LayerModel
{
public:
    LayerCylinder(const glm::vec3 &origin, float length, float radius,
                  const glm::vec3 &color);
    LayerCylinder(float length, float radius, const glm::vec3 &color);
    ~LayerCylinder();

    void setProperty(const glm::vec3 &origin, float length, float radius);
    void setProperty(float length, float radius);
    void setProperty(float length);

private:
    glm::vec3 _origin;
    float     _radius;
};

#endif // LAYER_CYLINDER_H_LF
