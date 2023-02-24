/**--------------------------------------------------------------------
 *																		
 *   				    Layer renderer library
 *																		
 * Description:													
 * This file is a part of lib_render_layer. You can redistribute it or
 * modify it to construct your own project. It is wellcome to use this 
 * library in your scientific research work.
 * 
 * @file 		layer_segment.h 
 * 
 * @brief 		Designed for rendering continuum segment
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
#ifndef LAYER_SEGMENT_H_LF
#define LAYER_SEGMENT_H_LF
#include "layer_model.h"
#include <lib_math/lib_math.h>


/**
 * @brief A class for rendering continuum segment.
 */
class LayerSegment : public LayerModel
{
public:
    /**
     * @brief Constructor of class LayerSegment.
     * @param length  The length of continuum segment.
     * @param theta   The bending angle of continuum segment.
     * @param delta   The bending direction of continuum segment.
     * @param radius  The radius of continuum segment.
     * @param color   The color of continuum segment.
     */
    LayerSegment(float length, float theta, float delta, float radius,
                 const glm::vec3 &color);
    ~LayerSegment();


    /**
     * @brief Change the property of current object.
     * @param length  The length of continuum segment.
     * @param theta   The bending angle of continuum segment.
     * @param delta   The bending direction of continuum segment.
     * @param radius  The radius of continuum segment.
     */
    void setProperty(float length, float theta, float delta, float radius);


    /**
     * @brief Change the property of current object.
     * @param length  The length of continuum segment.
     * @param theta   The bending angle of continuum segment.
     * @param delta   The bending direction of continuum segment.
     */
    void setProperty(float length, float theta, float delta);


    /**
     * @brief Change the property of current object.
     * @param q  The configuration of a continuum segment. The details could be
     * found at https://github.com/wlfrii/lib_math/blob/main/export/kine/continuum_configspc.h
     */
    void setProperty(const mmath::continuum::ConfigSpc &q);

private:
    float _radius;   //!< The radius of the continuum segment.
};

#endif // LAYER_CYLINDER_H_LF
