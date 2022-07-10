#ifndef LAYER_SEGMENT_H_LF
#define LAYER_SEGMENT_H_LF
#include "layer_model_base.h"

struct LayerSegmentProperty
{
    float     length;
    float     theta;
    float     delta;
    float     radius;
};

class LayerSegment : public LayerModelBase
{
public:
    LayerSegment(uint16_t width, uint16_t height, LayerType type,
                 LayerRenderMode mode, glm::vec3 color,
                 const LayerSegmentProperty& prop);
    ~LayerSegment();

    void setProperty(const LayerSegmentProperty& prop);
};

#endif // LAYER_CYLINDER_H_LF
