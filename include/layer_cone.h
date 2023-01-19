/**--------------------------------------------------------------------
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is a part of lib_render_layer. You can redistribute it or
 * modify it to construct your own project. It is wellcome to use this 
 * library in your scientific research work.
 * 
 * @file 		layer_cone.h
 * 
 * @brief 		Designed for rendering cone
 * 
 * @author		Longfei Wang
 * 
 * @date		2023/01/19
 * 
 * @license		
 * 
 * Copyright (C) 2022 Longfei Wang.
 * 
 * --------------------------------------------------------------------
 * Change History:                        
 * 
 * -------------------------------------------------------------------*/
#ifndef LAYER_CONE_H_LF
#define LAYER_CONE_H_LF
#include "layer_model.h"


/**
 * @brief A class for rendering circle
 */
class LayerCone : public LayerModel
{
public:
    /**
     * @brief Constructor of class LayerCone.
     * @param height  The radius of the cone
     * @param radius  The radius of the cone
     * @param color   The color of the cone
     * @param pose    The pose of the cone
     */
    LayerCone(float height, float radius, const glm::vec3& color,
              const glm::mat4& pose = glm::mat4(1.0f));


    ~LayerCone();


    /**
     * @brief Change the property for current object.
     * @param height  The radius of the cone
     * @param radius  The radius of the cone
     * @param pose    The pose of the cone
     */
    void setProperty(float height, float radius, const glm::mat4& pose);


    /**
     * @brief Change the property for current object.
     * @param height  The radius of the cone
     */
    void setHeight(float height);


private:
    glm::mat4 _pose;        //!< The pose of the cone
    float     _radius;      //!< The radius of the cone
    float     _height;      //!< The height of the cone
};

#endif // LAYER_CONE_H_LF
