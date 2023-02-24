#include "../include/layer_coordinate.h"
#include "./shapes/arrow_generator.h"
#include <gl_util.h>

namespace mlayer{

LayerCoordinate::LayerCoordinate(float length, float radius,
                                 const glm::mat4& pose)
    : LayerModel(LAYER_COORDINATE, glm::vec3(0.f, 0.f, 1.f))
    , _shader_x(new gl_util::Shader())
    , _vavbo_x(nullptr)
    , _shader_y(new gl_util::Shader())
    , _vavbo_y(nullptr)
{
    bool flag = _shader_x->load("./shaders/model.vs", "./shaders/model.fs");
    if(!flag) printf("LayerCoordinate read shader: %d\n", flag);
    flag = _shader_y->load("./shaders/model.vs", "./shaders/model.fs");
    if(!flag) printf("LayerCoordinate read shader: %d\n", flag);

    _shader_x->use();
    _shader_x->setVec3f("object_color", glm::vec3(1.f, 0.f, 0.f));
    _shader_x->setVec3f("light_color", _light_color);
    _shader_x->setVec3f("light_pos", _light_pos);
    _shader_y->use();
    _shader_y->setVec3f("object_color", glm::vec3(0.f, 1.f, 0.f));
    _shader_y->setVec3f("light_color", _light_color);
    _shader_y->setVec3f("light_pos", _light_pos);

    createCoordinate(length, radius, pose);
}


LayerCoordinate::~LayerCoordinate()
{
    if(_shader_x) {
        _shader_x->release();
        delete _shader_x;
        _shader_x = nullptr;
    }
    if(_vavbo_x) {
        _vavbo_x->release();
        delete _vavbo_x;
        _vavbo_x = nullptr;
    }
    if(_shader_y) {
        _shader_y->release();
        delete _shader_y;
        _shader_y = nullptr;
    }
    if(_vavbo_y) {
        _vavbo_y->release();
        delete _vavbo_y;
        _vavbo_y = nullptr;
    }
}


void LayerCoordinate::draw()
{
    auto &view = _view;

    // Draw z-axis
    _shader->use();
    _shader->setMat4f("projection", _projection);
    _shader->setMat4f("model", _model);
    _shader->setMat4f("view", view);
    _shader->setVec3f("view_pos", glm::vec3(view[3][0],view[3][1],view[3][2]));
    _vavbebo->bindVertexArray();
    glDrawArrays(GL_TRIANGLES, 0, _vert_num);

    // Draw x-axis
    _shader_x->use();
    _shader_x->setMat4f("projection", _projection);
    _shader_x->setMat4f("model", _model);
    _shader_x->setMat4f("view", view);
    _shader_x->setVec3f("view_pos", glm::vec3(view[3][0],view[3][1],view[3][2]));
    _vavbo_x->bindVertexArray();
    glDrawArrays(GL_TRIANGLES, 0, _vert_num_x);

    // Draw y-axis
    _shader_y->use();
    _shader_y->setMat4f("projection", _projection);
    _shader_y->setMat4f("model", _model);
    _shader_y->setMat4f("view", view);
    _shader_y->setVec3f("view_pos", glm::vec3(view[3][0],view[3][1],view[3][2]));
    _vavbo_y->bindVertexArray();
    glDrawArrays(GL_TRIANGLES, 0, _vert_num_y);
}


void LayerCoordinate::createCoordinate(float length, float radius,
                                       const glm::mat4 &pose)
{
    float cone_radius = radius * 2.f;
    float cone_height = cone_radius * 2.2f;

    // z-axis
    ArrowGenerator arrow_z(length, radius, cone_height, cone_radius, pose);
    auto data = arrow_z.vertices();
    _vavbebo->bind(&data[0].position.x, data.size() * sizeof(Vertex));
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(1);
    _vert_num = data.size();

    // x-axis
    glm::mat4 pose_x = pose;
    pose_x = glm::rotate(pose_x, glm::radians(90.f), glm::vec3(0.f, 1.f, 0.f));

    ArrowGenerator arrow_x(length, radius, cone_height, cone_radius, pose_x);
    data = arrow_x.vertices();
    _vavbo_x = new gl_util::VAVBEBO();
    _vavbo_x->bind(&data[0].position.x, data.size() * sizeof(Vertex));
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(1);
    _vert_num_x = data.size();

    // y-axis
    glm::mat4 pose_y = pose;
    pose_y = glm::rotate(pose_y, glm::radians(-90.f), glm::vec3(1.f, 0.f, 0.f));

    ArrowGenerator arrow_y(length, radius, cone_height, cone_radius, pose_y);
    data = arrow_y.vertices();
    _vavbo_y = new gl_util::VAVBEBO();
    _vavbo_y->bind(&data[0].position.x, data.size() * sizeof(Vertex));
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(1);
    _vert_num_y = data.size();
}

} // namespace::mlayer