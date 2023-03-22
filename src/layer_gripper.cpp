#include "../export/lib_layer_renderer/layer_gripper.h"
#include "../export/lib_layer_renderer/layer_cylinder.h"
#include "stl_reader.h"
#include "./shapes/cylinder_generator.h"
#include <gl_util.h>

namespace mlayer{

LayerGripper::LayerGripper(glm::vec3 color, GripperType gtype)
    : LayerModel(LAYER_GRIPPER, color)
    , gripper_type(gtype)
    , _vavbo_active(nullptr)
    , _vert_num_active(0)
    , _vavbo_active2(nullptr)
    , _vert_num_active2(0)
{
    _active_part = { 0, glm::vec3(0.f,0.f,0.f), glm::vec3(0.f,0.f,1.f) };

    _shader_ignore = new gl_util::Shader();
    bool flag = _shader_ignore->load("./shaders/model_ignore.vs",
                                "./shaders/model_ignore.fs");
    if(!flag) printf("LayerGripper read shader ignored: %d\n", flag);

    flag = loadModel();
    if(!flag) printf("LayerGripper read gripper model: %d\n", flag);
}


LayerGripper::~LayerGripper()
{
    if(_vavbo_active){
        _vavbo_active->release();
        delete  _vavbo_active;
        _vavbo_active = nullptr;
    }
    if(_vavbo_active2){
        _vavbo_active2->release();
        delete  _vavbo_active2;
        _vavbo_active2 = nullptr;
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


void LayerGripper::draw()
{
    glm::mat4 model = _global * _model;

    // Draw main part
    _shader->use();
    _shader->setMat4f("projection", _projection);
    _shader->setMat4f("model", model);
    _shader->setMat4f("view", _view);
    _vavbebo->bindVertexArray();
    glDrawArrays(GL_TRIANGLES, 0, _vert_num);

    // Draw active part
    glm::mat4 mat = model;
    mat = glm::translate(mat, _active_part.p);
    mat = glm::rotate(mat, _active_part.angle, _active_part.axis);
    mat = glm::translate(mat, -_active_part.p);
    _shader->setMat4f("model", mat);
    _vavbo_active->bindVertexArray();
    glDrawArrays(GL_TRIANGLES, 0, _vert_num_active);

    if(gripper_type == GRIPPER_TISSUE_GRASPING_FORCEPS){
        mat = model;
        mat = glm::translate(mat, _active_part.p);
        mat = glm::rotate(mat, -_active_part.angle, _active_part.axis);
        mat = glm::translate(mat, -_active_part.p);
        _shader->setMat4f("model", mat);
        _vavbo_active2->bindVertexArray();
        glDrawArrays(GL_TRIANGLES, 0, _vert_num_active2);
    }

    // Draw ignored part
    _shader_ignore->use();
    _shader_ignore->setMat4f("projection", _projection);
    _shader_ignore->setMat4f("model", model);
    _shader_ignore->setMat4f("view", _view);
    _vavbo_ignore->bindVertexArray();
    glDrawArrays(GL_TRIANGLES, 0, _vert_num_ignore);
}


void LayerGripper::setAngle(float angle)
{
    if(gripper_type == GRIPPER_BIPOLAR_GRASPING_FORCEPS){
        angle = -angle;
    }
    _active_part.angle = angle;
}


bool LayerGripper::loadModel()
{
    Vertices data[3];
    uint8_t part_num = 0;
    glm::vec3 origin(0.f);
    float length(0), radius(0);

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
        origin = glm::vec3(0.f, 0.f, -1.9+1.42 - 4.0);
        length = 8;
        radius = 3.7;
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
        origin = glm::vec3(0.f,0.f,-3.1f);
        length = 3.05 + 0.36;
        radius = 3.0 + 0.2;
        break;
    case GRIPPER_BIPOLAR_GRASPING_FORCEPS:
        if(!STLReader::getInstance()->read(STL_BGF_0, data[0]))
            return false;
        if(!STLReader::getInstance()->read(STL_BGF_1, data[1]))
            return false;
        _active_part.p = glm::vec3(-1.899f, 0.f, 5.753f);
        _active_part.axis = glm::vec3(0.f, 1.f, 0.f);
        part_num = 3;
        // Lengthen cylinder as tool body
        origin = glm::vec3(0.f,0.f,-3.0f);
        length = 3.05 + 0.1;
        radius = 3.55;
        break;
    case GRIPPER_TISSUE_GRASPING_FORCEPS:
        if(!STLReader::getInstance()->read(STL_TGF_0, data[0]))
            return false;
        if(!STLReader::getInstance()->read(STL_TGF_1, data[1]))
            return false;
        if(!STLReader::getInstance()->read(STL_TGF_2, data[2]))
            return false;
        _active_part.p = glm::vec3(0.f, 0.f, 8.703f);
        _active_part.axis = glm::vec3(0.f, 1.f, 0.f);
        part_num = 3;
        // Lengthen cylinder as tool body
        origin = glm::vec3(0.f,0.f,-3.0f);
        length = 3.05 + 0.1;
        radius = 3.25;
        break;
    default:
        return false;
    }

    // Bind fixed part
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

        if (!data[2].empty()) {
            _vavbo_active2 = new gl_util::VAVBEBO();
            _vavbo_active2->bind(&data[2][0].position.x, data[2].size() * sizeof(Vertex));
            glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
            glEnableVertexAttribArray(1);
            _vert_num_active2 = data[2].size();
        }
    }

    // Bind the ignored part
    if(part_num >= 3){
        CylinderGenerator cg(length, radius, origin);
        auto data = cg.vertices();

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

} // namespace::mlayer
