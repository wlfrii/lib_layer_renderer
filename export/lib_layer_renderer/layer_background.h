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

namespace mlayer{

/**
 * @brief The LayerBackground Data
 */
struct LayerBackgroundData
{
    LayerBackgroundData(unsigned char* data = nullptr, uint16_t width = 0,
                        uint16_t height = 0, uint8_t  channels = 0)
        : data(data), width(width), height(height), channels(channels)
    { }

    unsigned char* data; //!< The stereo image data
    uint16_t width;      //!< The width of the image
    uint16_t height;     //!< The height of the image
    uint8_t  channels;   //!< The channels of the image
};


/**
 * @brief A class for rendering window background
 */
class LayerBackground : public Layer
{
public:
    using Ptr = std::shared_ptr<LayerBackground>;


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
    void draw() override;


private:
    void bindTexture(const uint8_t *data, uint16_t w, uint16_t h, uint8_t c);
    void bindTextureMask(uint8_t *data, uint16_t w, uint16_t h);

    GLuint _texture;
    GLuint _texture_mask;

    glm::mat4 _transform;
};

} // namespace::mlayer
#endif // LAYER_BACKGROUND_H_LF
