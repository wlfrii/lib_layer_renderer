/**--------------------------------------------------------------------
 *																		
 *   				    Layer renderer library
 *																		
 * Description:													
 * This file is a part of lib_render_layer. You can redistribute it or
 * modify it to construct your own project. It is wellcome to use this 
 * library in your scientific research work.
 * 
 * @file 		layer_backgrounrd3D.h
 * 
 * @brief 		Designed for rendering 3D plane background
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
#ifndef LAYER_BACKGROUND3D_H_LF
#define LAYER_BACKGROUND3D_H_LF
#include "layer_background.h"


namespace mlayer{

/**
 * @brief A class for rendering window 3D plane background
 */
class LayerBackground3D : public LayerBackground
{
public:
    using Ptr = std::shared_ptr<LayerBackground3D>;


    LayerBackground3D(float bgwidth, float bgheight, float bgdepth);
    ~LayerBackground3D();


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


protected:
    void draw() override;
};

} // namespace::mlayer
#endif // LAYER_BACKGROUND3D_H_LF
