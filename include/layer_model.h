#ifndef LAYER_MODEL_H_LF
#define LAYER_MODEL_H_LF
#include "layer.h"

class LayerModel : public Layer
{
protected:
    LayerModel(uint16_t width, uint16_t height, LayerRenderMode mode,
               glm::vec3 color);

public:
    virtual ~LayerModel();

    void setModel(const glm::mat4& model);
    static void setView(const glm::mat4& view, bool is_right);
    static void setProjection(const glm::mat4& proj);

protected:
    virtual void draw(bool is_right) override;

    static glm::mat4  _projection;                 //!< Projection matrix
    static glm::mat4  _view[LAYER_RENDER_STEREO];  //!< View matrix

    size_t     _vert_num;               //!< The number of vertex

    glm::mat4  _model;                  //!< Model matrix
    glm::vec3  _object_color;
    glm::vec3  _light_color;
    glm::vec3  _light_pos;
};


#endif // LAYER_MODEL_H_LF
