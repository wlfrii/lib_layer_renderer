/**--------------------------------------------------------------------
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is a part of lib_render_layer. You can redistribute it or
 * modify it to construct your own project. It is wellcome to use this 
 * library in your scientific research work.
 * 
 * @file 		layer_texture3d.h 
 * 
 * @brief 		Designed for rendering 3D texture
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
#ifndef LAYER_TEXTURE3D_H_LF
#define LAYER_TEXTURE3D_H_LF
#include "layer_model.h"
#include <opencv2/opencv.hpp>

class SGM;


/**
 * @brief A class for rendering 3D texture.
 */
class LayerTexture3D : public LayerModel
{
public:
    /**
     * @brief Constructor of class LayerTexture3D.
     * @param left_tex   Left texture.
     * @param right_tex  Right texture.
     * @param fxy        Focal length of the camera.
     * @param cx         X coordinates of camera optical center.
     * @param cy         Y coordinates of camera optical center.
     * @param t          Distance between binocular camera.
     * @param pyd_times  Times of downsampling for the texture.
     */
    LayerTexture3D(const cv::Mat &left_tex, const cv::Mat &right_tex,
                   float fxy, float cx, float cy, float t,
                   uint8_t pyd_times = 2);
    ~LayerTexture3D();


    /**
     * @brief Update 3D texture
     * @param left_tex
     * @param right_tex
     */
    void update3DTexture(const cv::Mat &left_tex, const cv::Mat &right_tex);

private:
    float _fxy;           //!< Focal length of the camera.
    float _cx;            //!< X coordinates of camera optical center.
    float _cy;            //!< Y coordinates of camera optical center.
    float _t;             //!< Distance between binocular camera.

    SGM* _sgm;            //!< SGM object
    float *_disparity;    //!< Calculated disparity

    uint8_t _pyd_times;   //!< Times of downsampling
    uint16_t _width;      //!< Width of the texture
    uint16_t _height;     //!< height of the texture

    cv::Mat _texture;
    cv::Mat _left_gray;   //!< The temp image for store gray image of left_tex
    cv::Mat _right_gray;  //!< The temp image for store gray image of right_tex
};

#endif // LAYER_TEXTURE3D_H_LF
