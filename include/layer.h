#ifndef LAYER_H_LF
#define LAYER_H_LF
#include <cstdint>
#include <memory>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include "layer_util.h"

namespace gl_util{
    class Shader;
    class VAVBEBO;
}

/**
 * @brief Specify the render mode, 2D/Left/Right or 3D/Stereo
 */
using LayerRenderMode = unsigned char;
constexpr LayerRenderMode LAYER_RENDER_LEFT   = 0;
constexpr LayerRenderMode LAYER_RENDER_RIGHT  = 1;
constexpr LayerRenderMode LAYER_RENDER_STEREO = 2;
constexpr LayerRenderMode LAYER_RENDER_2D     = 0;
constexpr LayerRenderMode LAYER_RENDER_3D     = 2;


/**
 * @brief The Layer Type enum
 */
enum LayerType
{
    LAYER_BACKGROUND = 0,
    LAYER_CYLINDER,
    LAYER_GRIPPER_NH,        // Needle holder
    LAYER_SEGMENT
};


/**
 * @brief A base class designed for OpenGL rendering
 */
class Layer
{
protected:
    Layer(uint16_t width, uint16_t height, LayerType type, LayerRenderMode mode);

public:
    virtual ~Layer();

    /**
     * @brief Render current layer
     */
    virtual void render(LayerRenderMode mode);


    const uint16_t    width;     //!< Viewport width
    const uint16_t    height;    //!< Viewport height
    const LayerType       type;  //!< The layer type
    const LayerRenderMode mode;  //!< 2D or 3D

protected:
    /**
     * @brief draw  For the derivative class to draw.
     * @param is_right
     */
    virtual void draw(bool is_right);

    gl_util::Shader*     _shader;
    gl_util::VAVBEBO*    _vavbebo;
};

#endif // LAYER_H_LF
