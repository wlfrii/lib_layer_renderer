#include "../export/lib_layer_renderer/layer.h"
#include <gl_util.h>

namespace mlayer{

Layer::Layer(LayerType type)
    : type(type)
    , _shader(new gl_util::Shader())
    , _vavbebo(new gl_util::VAVBEBO())
    , _global(glm::mat4(1.0))
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


void Layer::render(const LayerViewPort &port)
{
    glViewport(port.x, port.y, port.width, port.height);
    draw();
}


void Layer::setModel(const glm::mat4&)
{

}


void Layer::setView(const glm::mat4&)
{

}


void Layer::setProjection(const glm::mat4&)
{

}


void Layer::setGlobal(const glm::mat4& global)
{
    _global = global;
}


void Layer::draw()
{

}

} // namespace::mlayer
