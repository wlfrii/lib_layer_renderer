/**--------------------------------------------------------------------
 *																		
 *   				    Layer renderer library
 *																		
 * Description:													
 * This file is a part of lib_render_layer. You can redistribute it or
 * modify it to construct your own project, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
 * PARTICULAR PURPOSE. See the GNU General Public License for more details.
 * 
 * @file 		layer.h 
 * 
 * @brief 		The base interface of the library
 * 
 * @author		Longfei Wang
 * 
 * @date		2022/04/01
 * 
 * @license		GPLv3
 * 
 * Copyright (C) 2021-Now Longfei Wang.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with lib_layer_renderer. If not, see <http://www.gnu.org/licenses/>
 * 
 * --------------------------------------------------------------------
 * Change History:                        
 * 
 * -------------------------------------------------------------------*/
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

namespace mlayer{

/**
 * @brief The Layer Type enum
 */
enum LayerType
{
    LAYER_BACKGROUND,     //!< For rendering 2D texture / image
    LAYER_CIRCLE,         //!< For rendering circle shape
    LAYER_CYLINDER,       //!< For rendering cylinder model
    LAYER_SEGMENT,        //!< For rendering continuum segment
    LAYER_CONE,           //!< For rendering cone model
    LAYER_COORDINATE,     //!< For rendering coordinate
    LAYER_END_EFFECTOR,   //!< For surigcal end-effector
    LAYER_TEXTURE3D,      //!< For rendering 3D texture / (colored) point cloud
    LAYER_UNKNOWN
};


/**
 * @brief A struct for specify the viewport to render layers
 */
struct LayerViewPort
{
    explicit LayerViewPort(int x, int y, uint16_t width, uint16_t height)
        : x(x), y(y), width(width), height(height) {}
    explicit LayerViewPort(uint16_t width, uint16_t height)
        : x(0), y(0), width(width), height(height) {}

    int x;         //!< Viewport x
    int y;         //!< Viewport y
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
    using Ptr = std::shared_ptr<Layer>;


    virtual ~Layer();

    /**
     * @brief Render current layer
     */
    virtual void render(const LayerViewPort &port);


    /**
     * @brief Set Model matrix
     * @param model
     */
    virtual void setModel(const glm::mat4& model);


    /**
     * @brief Set View matrix
     * @param view
     */
    virtual void setView(const glm::mat4& view);


    /**
     * @brief Set Projection matrix
     * @param proj
     */
    virtual void setProjection(const glm::mat4& proj);


    /**
     * @brief Get global transform
     * @return
     */
    virtual void setGlobal(const glm::mat4& global);


    const LayerType       type;         //!< Type of current layer


protected:
    /**
     * @brief draw  For the derivative class to draw.
     * @param is_right
     */
    virtual void draw();

    gl_util::Shader*  _shader;      //!< Shader for current layer
    gl_util::VAVBEBO* _vavbebo;     //!< VAO,VBO,EBO for current layer

    glm::mat4 _global;              //!< Global matrix (world frame)
    glm::mat4 _model;               //!< Model matrix
    glm::mat4 _projection;          //!< Projection matrix
    glm::mat4 _view;                //!< View matrix
};

} // namespace::mlayer
#endif // LAYER_H_LF
