/**--------------------------------------------------------------------
 *																		
 *   				    Layer renderer library
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
 * Copyright (C) 2021-Now Longfei Wang.
 * 
 * --------------------------------------------------------------------
 * Change History:                        
 * 
 * -------------------------------------------------------------------*/
#ifndef LAYER_CYLINDER_H_LF
#define LAYER_CYLINDER_H_LF
#include "layer_model.h"

namespace mlayer{

/**
 * @brief A class for rendering cylinder
 */
class LayerCylinder : public LayerModel
{
public:
    using Ptr = std::shared_ptr<LayerCylinder>;


    /**
     * @brief Constructor of class LayerCylinder.
     * @param length  The length of the cylinder
     * @param radius  The radius of the cylinder
     * @param color   The color of the cylinder
     * @param pose  The origin of the cylinder
     */
    LayerCylinder(float length, float radius, const glm::vec3 &color,
                  const glm::mat4& pose = glm::mat4(1.f));


    ~LayerCylinder();


    /**
     * @brief Change the property for current object.
     * @param length  The length of the cylinder
     * @param radius  The radius of the cylinder
     * @param pose  The origin of the cylinder
     */
    void setProperty(float length, float radius, const glm::mat4 &pose);


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
    glm::mat4 _pose;      //!< The origin of the cylinder
    float     _length;    //!< The length of the cylinder
    float     _radius;    //!< The radius of the cylinder
};

} // namespace::mlayer
#endif // LAYER_CYLINDER_H_LF
