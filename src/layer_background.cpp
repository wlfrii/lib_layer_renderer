#include "layer_background.h"
#include <gl_util.h>
#include <global.h>


LayerBackground::LayerBackground(uint16_t width, uint16_t height, LayerType type, LayerRenderMode mode)
    : Layer(width, height, type, mode)
    , _has_texture(false)
    , _has_texture_mask(false)
{
    _shader = new gl_util::Shader();
    bool flag = _shader->load("./shaders/texture.vs", "./shaders/texture.fs");
    EV_LOG("LayerBackground read shader: %d\n", flag);
    _shader->use();
    _shader->setInt("image", 0);
    _shader->setInt("mask", 1);

    float vertices[] = {
        // positions          // texture coords
         1.0f,  1.0f, 1.0f,   1.0f, 1.0f,  // top right
         1.0f, -1.0f, 1.0f,   1.0f, 0.0f,  // bottom right
        -1.0f, -1.0f, 1.0f,   0.0f, 0.0f,  // bottom left
        -1.0f,  1.0f, 1.0f,   0.0f, 1.0f   // top left
    };
    unsigned int indices[] = {
        0, 1, 3, // first triangle
        1, 2, 3  // second triangle
    };
    _vavbebo = new gl_util::VAVBEBO();
    _vavbebo->bind(vertices, sizeof(vertices), indices, sizeof(indices));
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
}


LayerBackground::~LayerBackground()
{

}


void LayerBackground::updateData(const LayerBackgroundData* data)
{
    assert(data->mode == this->mode);

    assert(data->data[0]);
    bindTexture(data->data[0], data->width, data->height,
            data->channels, 0);

    if(data->mode == LAYER_RENDER_3D){
        assert(data->data[1]);
        bindTexture(data->data[1], data->width, data->height, data->channels, 1);
    }
    _has_texture = true;
}


void LayerBackground::updateMask(const LayerBackgroundData* data)
{
    assert(data->mode == this->mode && data->channels == 1);

    assert(data->data[0]);
    bindTextureMask(data->data[0], data->width, data->height, 0);

    if(data->mode == LAYER_RENDER_3D){
        assert(data->data[1]);
        bindTextureMask(data->data[1], data->width, data->height, 1);
    }
    _has_texture_mask = true;
}


void LayerBackground::draw(bool is_right)
{
    assert(_has_texture);

    _shader->use();
    _vavbebo->bindVertexArray();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, _texture[is_right]);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, _texture_mask[is_right]);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}


void LayerBackground::bindTexture(uint8_t *data, uint16_t w, uint16_t h,
                                  uint16_t c, bool is_right)
{
    if(!_has_texture){
        glGenTextures(1, &_texture[is_right]);
    }
    glBindTexture(GL_TEXTURE_2D, _texture[is_right]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    GLenum fmt = GL_RGB;
    if(c == 4) fmt = GL_RGBA;

    glTexImage2D(GL_TEXTURE_2D, 0, fmt, w, h, 0, fmt, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
}


void LayerBackground::bindTextureMask(uint8_t *data, uint16_t w, uint16_t h,
                                  bool is_right)
{
    if(!_has_texture_mask){
        glGenTextures(1, &_texture_mask[is_right]);
    }
    glBindTexture(GL_TEXTURE_2D, _texture_mask[is_right]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    GLenum fmt = GL_RED;

    glTexImage2D(GL_TEXTURE_2D, 0, fmt, w, h, 0, fmt, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
}
