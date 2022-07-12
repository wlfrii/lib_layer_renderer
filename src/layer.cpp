#include "layer.h"
#include <gl_util.h>


Layer::Layer(LayerType type)
    : type(type)
    , _shader(new gl_util::Shader())
    , _vavbebo(new gl_util::VAVBEBO())
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


void Layer::render(const LayerViewPort &port, LayerRenderMode mode)
{
    if(mode == LAYER_RENDER_LEFT){
        glViewport(port.x, port.y, port.width, port.height);
        draw(LAYER_RENDER_LEFT);
    }
    else if(mode == LAYER_RENDER_RIGHT){
        glViewport(port.x, port.y, port.width, port.height);
        draw(LAYER_RENDER_RIGHT);
    }
    else{
        // Draw left
        glViewport(port.x, port.y, port.width / 2, port.height);
        draw(LAYER_RENDER_LEFT);

        // Draw right
        glViewport(port.width / 2, 0, port.width / 2, port.height);
        draw(LAYER_RENDER_RIGHT);
    }
}


void Layer::draw(bool is_right)
{

}
