/**--------------------------------------------------------------------
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is a part of lib_render_layer. You can redistribute it or
 * modify it to construct your own project. It is wellcome to use this 
 * library in your scientific research work.
 * 
 * @file 		layer_coordinate.h
 * 
 * @brief 		Designed for rendering coordinate system
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
#ifndef LAYER_COORDINATE_H_LF
#define LAYER_COORDINATE_H_LF
#include "layer_model.h"


/**
 * @brief A class for rendering coordinate system
 */
class LayerCoordinate : public LayerModel
{
public:
    /**
     * @brief Constructor of class LayerCone.
     * @param length  The length of the coordinate axis
     * @param radius  The radius of the coordinate axis
     * @param pose    The pose of the coordinate
     */
    LayerCoordinate(float length, float radius,
                    const glm::mat4& pose = glm::mat4(1.0f));


    ~LayerCoordinate();


protected:
    /* Draw the vertices */
    void draw(bool is_right) override;

private:    
    gl_util::VAVBEBO* _vavbo_x_;
    size_t            _vert_num_x;
    gl_util::VAVBEBO* _vavbo_y;
    size_t            _vert_num_y;
};

#endif // LAYER_COORDINATE_H_LF