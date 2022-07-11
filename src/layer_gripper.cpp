#include "../include/layer_gripper.h"
#include "../include/layer_cylinder.h"
#include "stl_reader.h"
#include "./shapes/cylinder_generator.h"
#include <gl_util.h>
#include <global.h>

LayerGripper::LayerGripper(uint16_t width, uint16_t height,
                           LayerRenderMode mode, glm::vec3 color,
                           GripperType gtype)
    : LayerModel(width, height, mode, LAYER_GRIPPER_NH, color)
    , gripper_type(gtype)
    , _vavbo_active(nullptr)
    , _vert_num_active(0)
{
    _active_part = { 0, glm::vec3(0.f,0.f,0.f), glm::vec3(0.f,0.f,1.f) };

    _shader_ignore = new gl_util::Shader();
    bool flag = _shader_ignore->load("./shaders/model_ignore.vs",
                                "./shaders/model_ignore.fs");
    EV_LOG("LayerGripper read shader ignored: %d\n", flag);

    flag = loadModel();
    EV_LOG("LayerGripper read gripper model: %d\n", flag);
}


LayerGripper::~LayerGripper()
{
    if(_vavbo_active){
        _vavbo_active->release();
        delete  _vavbo_active;
        _vavbo_active = nullptr;
    }
    if(_shader_ignore){
        _shader_ignore->release();
        delete  _shader_ignore;
        _shader_ignore = nullptr;
    }
    if(_vavbo_ignore){
        _vavbo_ignore->release();
        delete  _vavbo_ignore;
        _vavbo_ignore = nullptr;
    }
}


void LayerGripper::draw(bool is_right)
{
    // Draw main part
    _shader->use();
    _shader->setMat4f("projection", _projection);
    _shader->setMat4f("model", _model);
    _shader->setMat4f("view", _view[is_right]);
    _vavbebo->bindVertexArray();
    glDrawArrays(GL_TRIANGLES, 0, _vert_num);

    // Draw active part
    glm::mat4 mat = _model;
    mat = glm::translate(mat, _active_part.p);
    mat = glm::rotate(mat, _active_part.angle, _active_part.axis);
    mat = glm::translate(mat, -_active_part.p);
    _shader->setMat4f("model", mat);
    _vavbo_active->bindVertexArray();
    glDrawArrays(GL_TRIANGLES, 0, _vert_num_active);

    // Draw ignored part
    _shader_ignore->use();
    _shader_ignore->setMat4f("projection", _projection);
    _shader_ignore->setMat4f("model", _model);
    _shader_ignore->setMat4f("view", _view[is_right]);
    _vavbo_ignore->bindVertexArray();
    glDrawArrays(GL_TRIANGLES, 0, _vert_num_ignore);
}


void LayerGripper::setAngle(float angle)
{
    _active_part.angle = angle;
}


bool LayerGripper::loadModel()
{
    Vertices data[2];
    uint8_t part_num = 0;
    LayerCylinderProperty prop;
    switch (gripper_type) {
    case GRIPPER_NEEDLE_HOLDER_SIMPLIFIED:
        if(!STLReader::getInstance()->read(STL_NH_0_SIMPLIFIED, data[0]))
            return false;
        if(!STLReader::getInstance()->read(STL_NH_1_SIMPLIFIED, data[1]))
            return false;
        _active_part.p = glm::vec3(0, -1.2, 5.7);
        _active_part.axis = glm::vec3(1.f, 0.f, 0.f);
        part_num = 3;
        // Lengthen cylinder as tool body
        //prop = {3.05, 2.85, glm::vec3(0.f,0.f,-1.9f)};
        prop = { glm::vec3(0.f, 0.f, -1.9+1.42 - 4.0), 8, 3.7 };
        break;
    case GRIPPER_NEEDLE_HOLDER:
        if(!STLReader::getInstance()->read(STL_NH_0, data[0]))
            return false;
        if(!STLReader::getInstance()->read(STL_NH_1, data[1]))
            return false;
        _active_part.p = glm::vec3(0, -1.201, 6.10);
        _active_part.axis = glm::vec3(1.f, 0.f, 0.f);
        part_num = 3;
        // Lengthen cylinder as tool body
        prop = { glm::vec3(0.f,0.f,-3.1f), 3.05, 3.0 };
        break;
    default:
        return false;
    }

    // Bind fixed part
    _vavbebo = new gl_util::VAVBEBO();
    _vavbebo->bind(&data[0][0].position.x, data[0].size() * sizeof(Vertex));
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(1);
    _vert_num = data[0].size();

    // Bind active part
    if(part_num >= 2){
        _vavbo_active = new gl_util::VAVBEBO();
        _vavbo_active->bind(&data[1][0].position.x, data[1].size() * sizeof(Vertex));
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
        glEnableVertexAttribArray(1);
        _vert_num_active = data[1].size();
    }

    // Bind the ignored part
    if(part_num >= 3){
        int num = (int)prop.radius*3.1415926*2 / 1.f;
        CylinderGenerator cg(num, prop.origin, prop.length, prop.radius);
        auto data = cg.result();

        _vavbo_ignore = new gl_util::VAVBEBO();
        _vavbo_ignore->bind(&data[0].position.x, data.size() * sizeof(Vertex));
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
        glEnableVertexAttribArray(1);
        _vert_num_ignore = data.size();
    }

    return true;
}


