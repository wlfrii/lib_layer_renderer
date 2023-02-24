#include "../export/lib_layer_renderer/layer_background.h"
#include <gl_util.h>

namespace mlayer{

LayerBackground::LayerBackground()
    : Layer(LAYER_BACKGROUND)
{
    // Load shader
    bool flag = _shader->load("./shaders/texture.vs", "./shaders/texture.fs");
    if(!flag) printf("LayerBackground read shader: %d\n", flag);

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

    // Bind VAVBEBO
    _vavbebo->bind(vertices, sizeof(vertices), indices, sizeof(indices));
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glGenTextures(1, &_texture);
    glGenTextures(1, &_texture_mask);
}


LayerBackground::~LayerBackground()
{

}


void LayerBackground::updateData(const LayerBackgroundData* data)
{
    if(data->data){
        bindTexture(data->data, data->width, data->height,
                    data->channels);
    }
}


void LayerBackground::updateMask(const LayerBackgroundData* data)
{
    if(data->channels != 1) std::abort();

    if(data->data){
        bindTextureMask(data->data, data->width, data->height);
    }
}


void LayerBackground::draw()
{
    _shader->use();
    _vavbebo->bindVertexArray();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, _texture);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, _texture_mask);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}


void LayerBackground::bindTexture(const uint8_t *data, uint16_t w, uint16_t h,
                                  uint8_t c)
{
    glBindTexture(GL_TEXTURE_2D, _texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    GLenum fmt = GL_RGB;
    if(c == 4){ fmt = GL_RGBA; }

    glTexImage2D(GL_TEXTURE_2D, 0, fmt, w, h, 0, fmt, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
}


void LayerBackground::bindTextureMask(uint8_t *data, uint16_t w, uint16_t h)
{
    glBindTexture(GL_TEXTURE_2D, _texture_mask);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    GLenum fmt = GL_RED;

    glTexImage2D(GL_TEXTURE_2D, 0, fmt, w, h, 0, fmt, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
}

} // namespace::mlayer
