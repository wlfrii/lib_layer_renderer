#include "layer.h"
#include <gl_util.h>


Layer::Layer(uint16_t width, uint16_t height, LayerRenderMode mode, LayerType type)
    : width(width)
    , height(height)
    , mode(mode)
    , type(type)
    , _shader(nullptr)
    , _vavbebo(nullptr)
{

}


Layer::~Layer()
{
    if(_shader){
        _shader->release();
        delete  _shader;
        _shader = nullptr;
    }

    if(_vavbebo){
        _vavbebo->release();
        delete _vavbebo;
        _vavbebo = nullptr;
    }
}


void Layer::render(LayerRenderID id)
{
    if(mode == LAYER_RENDER_2D || id == LAYER_RENDER_LEFT){
        glViewport(0, 0, width, height);
        draw(LAYER_RENDER_LEFT);
    }
    else if(id == LAYER_RENDER_RIGHT){
        glViewport(0, 0, width, height);
        draw(LAYER_RENDER_RIGHT);
    }
    else{
        // Draw left
        glViewport(0, 0, width / 2, height);
        draw(LAYER_RENDER_LEFT);

        // Draw right
        glViewport(width / 2, 0, width / 2, height);
        draw(LAYER_RENDER_RIGHT);
    }
}


void Layer::draw(bool is_right)
{

}
