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
 * @brief Specify the render index, Left/Right for 2D mode or Stereo 3D mode.
 */
enum LayerRenderMode
{
    LAYER_RENDER_LEFT   = 0,
    LAYER_RENDER_RIGHT  = 1,
    LAYER_RENDER_STEREO = 2
};


/**
 * @brief The Layer Type enum
 */
enum LayerType
{
    LAYER_BACKGROUND,
    LAYER_CYLINDER,
    LAYER_SEGMENT,
    LAYER_GRIPPER_NH,        // Needle holder
    LAYER_TEXTURE3D,
    LAYER_UNKNOWN
};


/**
 * @brief The LayerViewRect struct
 */
struct LayerViewPort
{
    explicit LayerViewPort(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
        : x(x), y(y), width(width), height(height) {}
    explicit LayerViewPort(uint16_t width, uint16_t height)
        : x(0), y(0), width(width), height(height) {}

    uint16_t x;         //!< Viewport x
    uint16_t y;         //!< Viewport y
    uint16_t width;     //!< Viewport width
    uint16_t height;    //!< Viewport height
};


/**
 * @brief A base class designed for OpenGL rendering
 */
class Layer
{
protected:
    /**
     * @brief Layer base class's constructor
     * @param mode  Specify 2D/3D mode
     * @param type  Store the LayerType
     *
     */
    Layer(LayerType type);

public:
    virtual ~Layer();

    /**
     * @brief Render current layer
     */
    virtual void render(const LayerViewPort &port, LayerRenderMode mode = LAYER_RENDER_LEFT);

    const LayerType       type;

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
