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
 * @brief Specify the render mode, 2D/3D
 */
enum LayerRenderMode
{
    LAYER_RENDER_2D   = 0,
    LAYER_RENDER_3D   = 2
};

/**
 * @brief Specify the render index, Left/Right for 2D mode or Stereo 3D mode.
 */
enum LayerRenderID
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
    LAYER_UNKNOWN
};


/**
 * @brief A base class designed for OpenGL rendering
 */
class Layer
{
protected:
    /**
     * @brief Layer base class's constructor
     * @param width  The width of layer view
     * @param height  The height of layer view
     */
    Layer(uint16_t width, uint16_t height, LayerRenderMode mode, LayerType type);

public:
    virtual ~Layer();

    /**
     * @brief Render current layer
     */
    virtual void render(LayerRenderID id = LAYER_RENDER_LEFT);

    const uint16_t        width;     //!< Viewport width
    const uint16_t        height;    //!< Viewport height
    const LayerRenderMode mode;
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
