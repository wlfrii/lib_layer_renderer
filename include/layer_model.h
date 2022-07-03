#ifndef LAYER_MODEL_H_LF
#define LAYER_MODEL_H_LF
#include "layer.h"

class LayerModel : public Layer
{
public:
    LayerModel(uint16_t width, uint16_t height, LayerType type,
               LayerRenderMode mode, glm::vec3 color);
    virtual ~LayerModel();

    void setView(const glm::mat4& view, bool is_right);
    void setModel(const glm::mat4& model);
    void setProjection(const glm::mat4& proj);

protected:
    virtual void draw(bool is_right) override;

    size_t     _vert_num;               //!< The number of vertex

    glm::mat4  _view[LAYER_RENDER_3D];  //!< View matrix
    glm::mat4  _model;                  //!< Model matrix
    glm::mat4  _projection;             //!< Projection matrix

    glm::vec3  _object_color;
    glm::vec3  _light_color;
    glm::vec3  _light_pos;
};


#endif // LAYER_MODEL_H_LF
