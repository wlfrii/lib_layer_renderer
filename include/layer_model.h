/**--------------------------------------------------------------------
 *																		
 *   				    Layer renderer library
 *																		
 * Description:													
 * This file is a part of lib_render_layer. You can redistribute it or
 * modify it to construct your own project. It is wellcome to use this 
 * library in your scientific research work.
 * 
 * @file 		layer_model.h 
 * 
 * @brief 		Base class for 3D object
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
#ifndef LAYER_MODEL_H_LF
#define LAYER_MODEL_H_LF
#include "layer.h"


/**
 * @brief A base class for rendering model that requires Model, View, and
 * Projection matrix.
 */
class LayerModel : public Layer
{
protected:
    /**
     * @brief Protected constructor of class LayerModel.
     * @param type  Type of the layer.
     * @param color Color of the model
     */
    LayerModel(LayerType type, const glm::vec3 &color);

public:
    virtual ~LayerModel();

    /**
     * @brief Set Model matrix
     * @param model
     */
    void setModel(const glm::mat4& model) override;


    /**
     * @brief Set View matrix
     * @param view
     */
    void setView(const glm::mat4& view) override;


    /**
     * @brief Set Projection matrix
     * @param proj
     */
    void setProjection(const glm::mat4& proj) override;


    /**
     * @brief Change the object color
     * @param color
     */
    void setColor(const glm::vec3 &object_color);


protected:
    /* Draw the vertices */
    virtual void draw() override;

    glm::mat4  _projection;             //!< Projection matrix
    glm::mat4  _view;                   //!< View matrix

    size_t     _vert_num;               //!< The number of vertex

    glm::mat4  _model;                  //!< Model matrix
    glm::vec3  _object_color;           //!< Color of the model
    glm::vec3  _light_color;            //!< Color of the light source
    glm::vec3  _light_pos;              //!< Position of the light source
};


#endif // LAYER_MODEL_H_LF
