#include "../export/lib_layer_renderer/layer_model.h"
#include <gl_util.h>

namespace mlayer{

LayerModel::LayerModel(LayerType type, const glm::vec3 &color)
    : Layer(type)
    , _projection(glm::mat4(1.f))
    , _view(glm::mat4(1.f))
    , _object_color(color)
    , _light_color(glm::vec3(1.f, 1.f, 1.f))
    , _light_pos(glm::vec3(0.f, 0.f, 0.f))
{
    if(type == LAYER_TEXTURE3D){
        bool flag = _shader->load("./shaders/texture_3d.vs", "./shaders/texture_3d.fs");
        if(!flag) printf("LayerTexture3D read shader: %d\n", flag);
    }
    else {
        bool flag = _shader->load("./shaders/model.vs", "./shaders/model.fs");
        if(!flag) printf("LayerModel read shader: %d\n", flag);
    }

    _shader->use();
    _shader->setVec3f("object_color", _object_color);
    _shader->setVec3f("light_color", _light_color);
    _shader->setVec3f("light_pos", _light_pos);
}


LayerModel::~LayerModel()
{

}


void LayerModel::setModel(const glm::mat4& model)
{
    _model = model;
}


void LayerModel::setView(const glm::mat4& view)
{
    _view = view;
}


void LayerModel::setProjection(const glm::mat4& proj)
{
    _projection = proj;
}


void LayerModel::setColor(const glm::vec3 &object_color)
{
    _object_color = object_color;

    _shader->use();
    _shader->setVec3f("object_color", _object_color);
}


void LayerModel::draw()
{
    _shader->use();
    _shader->setMat4f("projection", _projection);
    _shader->setMat4f("model", _model);
    _shader->setMat4f("view", _view);
    auto &view = _view;
    _shader->setVec3f("view_pos", glm::vec3(view[3][0],view[3][1],view[3][2]));
    _vavbebo->bindVertexArray();
    glDrawArrays(GL_TRIANGLES, 0, _vert_num);
}

} // namespace::mlayer
