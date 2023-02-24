/**--------------------------------------------------------------------
 *																		
 *   				    Layer renderer library
 *																		
 * Description:													
 * This file is a part of lib_render_layer. You can redistribute it or
 * modify it to construct your own project. It is wellcome to use this 
 * library in your scientific research work.
 * 
 * @file 		layer.h 
 * 
 * @brief 		The base interface of the library
 * 
 * @author		Longfei Wang
 * 
 * @date		2022/04/01
 * 
 * @license		
 * 
 * Copyright (C) 2021-Now Longfei Wang.
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
    LAYER_GRIPPER,        //!< For surigcal gripper
    LAYER_TEXTURE3D,      //!< For rendering 3D texture / (colored) point cloud
    LAYER_UNKNOWN
};


/**
 * @brief A struct for specify the viewport to render layers
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


    const LayerType       type;         //!< Type of current layer


protected:
    /**
     * @brief draw  For the derivative class to draw.
     * @param is_right
     */
    virtual void draw();

    gl_util::Shader*     _shader;       //!< Shader for current layer
    gl_util::VAVBEBO*    _vavbebo;      //!< VAO,VBO,EBO for current layer
};

} // namespace::mlayer
#endif // LAYER_H_LF
