#ifndef LAYER_SEGMENT_H_LF
#define LAYER_SEGMENT_H_LF
#include "layer_model.h"
#include <lib_math/lib_math.h>


struct LayerSegmentProperty
{
    float     length;
    float     theta;
    float     delta;
    float     radius;
};

class LayerSegment : public LayerModel
{
public:
    LayerSegment(float length, float theta, float delta, float radius,
                 const glm::vec3 &color);
    ~LayerSegment();

    void setProperty(float length, float theta, float delta, float radius);
    void setProperty(float length, float theta, float delta);
    void setProperty(const mmath::continuum::ConfigSpc &q);

private:
    float _radius;
};

#endif // LAYER_CYLINDER_H_LF
