/**--------------------------------------------------------------------
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is a part of lib_render_layer. You can redistribute it or
 * modify it to construct your own project. It is wellcome to use this 
 * library in your scientific research work.
 * 
 * @file 		layer_cylinder.h 
 * 
 * @brief 		Designed for rendering cylinder
 * 
 * @author		Longfei Wang
 * 
 * @date		2022/04/01
 * 
 * @license		
 * 
 * Copyright (C) 2022 Longfei Wang.
 * 
 * --------------------------------------------------------------------
 * Change History:                        
 * 
 * -------------------------------------------------------------------*/
#ifndef LAYER_CYLINDER_H_LF
#define LAYER_CYLINDER_H_LF
#include "layer_model.h"


/**
 * @brief A class for rendering cylinder
 */
class LayerCylinder : public LayerModel
{
public:
    /**
     * @brief Constructor of class LayerCylinder.
     * @param origin  The origin of the cylinder
     * @param length  The length of the cylinder
     * @param radius  The radius of the cylinder
     * @param color   The color of the cylinder
     */
    LayerCylinder(const glm::vec3 &origin, float length, float radius,
                  const glm::vec3 &color);

    /**
     * @brief Constructor of class LayerCylinder, with default (0,0,0) origin.
     * @param length  The length of the cylinder
     * @param radius  The radius of the cylinder
     * @param color   The color of the cylinder
     */
    LayerCylinder(float length, float radius, const glm::vec3 &color);
    ~LayerCylinder();


    /**
     * @brief Change the property for current object.
     * @param origin  The origin of the cylinder
     * @param length  The length of the cylinder
     * @param radius  The radius of the cylinder
     */
    void setProperty(const glm::vec3 &origin, float length, float radius);


    /**
     * @brief Change the property for current object.
     * @param length  The length of the cylinder
     * @param radius  The radius of the cylinder
     */
    void setProperty(float length, float radius);


    /**
     * @brief Change the property for current object.
     * @param length  The length of the cylinder
     */
    void setProperty(float length);

private:
    glm::vec3 _origin;      //!< The origin of the cylinder
    float     _radius;      //!< The radius of the cylinder
};

#endif // LAYER_CYLINDER_H_LF
