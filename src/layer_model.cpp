#include "layer_model.h"
#include <gl_util.h>
#include <global.h>

LayerModel::LayerModel(uint16_t width, uint16_t height, LayerType type,
                       LayerRenderMode mode, glm::vec3 color)
    : Layer(width, height, type, mode)
    , _object_color(color)
    , _light_color(glm::vec3(1.f, 1.f, 1.f))
    , _light_pos(glm::vec3(0.f, -50.f, 0.f))
{
    _shader = new gl_util::Shader();
    bool flag = _shader->load("./shaders/model.vs", "./shaders/model.fs");
    EV_LOG("LayerModel read shader: %d\n", flag);

    _shader->use();
    _shader->setVec3f("object_color", _object_color);
    _shader->setVec3f("light_color", _light_color);
    _shader->setVec3f("light_pos", _light_pos);
}


LayerModel::~LayerModel()
{

}


void LayerModel::draw(bool is_right)
{
    _shader->use();
    _shader->setMat4f("projection", _projection);
    _shader->setMat4f("model", _model);
    auto &view = _view[is_right];
    _shader->setMat4f("view", _view[is_right]);
    _shader->setVec3f("view_pos", glm::vec3(view[3][0],view[3][1],view[3][2]));
    //_shader->setVec3f("light_pos", glm::vec3(view[3][0],view[3][1],view[3][2]));
    _vavbebo->bindVertexArray();
    glDrawArrays(GL_TRIANGLES, 0, _vert_num);
}


void LayerModel::setView(const glm::mat4& view, bool is_right)
{
    _view[is_right] = view;
}


void LayerModel::setModel(const glm::mat4& model)
{
    _model = model;
}


void LayerModel::setProjection(const glm::mat4& proj)
{
    _projection = proj;
}
