/**--------------------------------------------------------------------
 *																		
 *   				    Layer renderer library
 *																		
 * Description:													
 * This file is a part of lib_render_layer. You can redistribute it or
 * modify it to construct your own project. It is wellcome to use this 
 * library in your scientific research work.
 * 
 * @file 		layer_backgrounr.h 
 * 
 * @brief 		Designed for rendering 2D/3D background
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
#ifndef LAYER_BACKGROUND_H_LF
#define LAYER_BACKGROUND_H_LF
#include "layer.h"
#include <string>


/**
 * @brief The LayerBackground Data
 */
struct LayerBackgroundData
{
    LayerBackgroundData() : width(0), height(0), channels(0)
    {
        data[LAYER_RENDER_LEFT] = nullptr;
        data[LAYER_RENDER_RIGHT] = nullptr;
    }

    unsigned char* data[LAYER_RENDER_STEREO]; //!< The stereo image data
    uint16_t width;                           //!< The width of the image
    uint16_t height;                          //!< The height of the image
    uint8_t  channels;                        //!< The channels of the image
};


/**
 * @brief A class for rendering window background
 */
class LayerBackground : public Layer
{
public:
    LayerBackground();
    ~LayerBackground();

    /**
     * @brief Update the background image data
     * @param data  Either monocular or binocular image data.
     */
    void updateData(const LayerBackgroundData *data);


    /**
     * @brief Update the mask for background
     * @param data  Should be single channel mask with same size as image data.
     */
    void updateMask(const LayerBackgroundData *data);

protected:
    void draw(bool is_right) override;


private:
    void bindTexture(const uint8_t *data, uint16_t w, uint16_t h, uint8_t c,
                     bool is_right);
    void bindTextureMask(uint8_t *data, uint16_t w, uint16_t h, bool is_right);

    GLuint _texture[LAYER_RENDER_STEREO];
    GLuint _texture_mask[LAYER_RENDER_STEREO];
};

#endif // LAYER_BACKGROUND_H_LF
