#ifndef LAYER_BACKGROUND_H_LF
#define LAYER_BACKGROUND_H_LF
#include "layer.h"
#include <string>


/**
 * @brief The LayerBackground Data
 */
class LayerBackgroundData
{
public:
    LayerBackgroundData(LayerRenderMode mode)
        : mode(mode) {}
    ~LayerBackgroundData() {}

    const LayerRenderMode mode;

    unsigned char* data[LAYER_RENDER_3D];   //!< The stereo image data
    uint16_t width;                         //!< The width of the image
    uint16_t height;                        //!< The height of the image
    uint8_t  channels;                      //!< The channels of the image
};


/**
 * @brief A class for rendering window background
 */
class LayerBackground : public Layer
{
public:
    LayerBackground(uint16_t width, uint16_t height, LayerRenderMode mode);
    ~LayerBackground();

    void updateData(const LayerBackgroundData *data);
    void updateMask(const LayerBackgroundData *data);

protected:
    void draw(bool is_right) override;

private:
    void bindTexture(const uint8_t *data, uint16_t w, uint16_t h, uint8_t c,
                     bool is_right);
    void bindTextureMask(uint8_t *data, uint16_t w, uint16_t h, bool is_right);

    bool   _has_texture;
    GLuint _texture[LAYER_RENDER_3D];
    bool   _has_texture_mask;
    GLuint _texture_mask[LAYER_RENDER_3D];
};

#endif // LAYER_BACKGROUND_H_LF
