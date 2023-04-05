#include "../include/lib_layer_renderer/layer_texture3d.h"
#include <lib_math/lib_math.h>
#include <gl_util.h>
#include "layer_define.h"

namespace mlayer{


LayerTexture3D::LayerTexture3D()
    : LayerModel(LAYER_TEXTURE3D, glm::vec3(0.f, 0.f, 0.f))
{

}


LayerTexture3D::~LayerTexture3D()
{

}


void LayerTexture3D::updateVertex3D(std::vector<Vertex3D>& vertices_3d)
{
    _draw_type = 1;

    // Now, vertices are arranged triangle vertices
    _vavbebo->bind(&vertices_3d[0].position.x, vertices_3d.size() * sizeof(Vertex3D));
    // position attribute
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // texture attribute
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(1);
    _vert_num = vertices_3d.size();
}


void LayerTexture3D::updateVertex(std::vector<Vertex3D>& vertices_3d)
{
    _draw_type = 2;

    // Now, vertices are arranged triangle vertices
    _vavbebo->bind(&vertices_3d[0].position.x, vertices_3d.size() * sizeof(Vertex3D));
    // position attribute
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // texture attribute
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
    glEnableVertexAttribArray(1);
    _vert_num = vertices_3d.size();
}



void LayerTexture3D::draw()
{
    _shader->use();
    _shader->setMat4f("projection", _projection);
    _shader->setMat4f("model", _global * _model);
    _shader->setMat4f("view", _view);
    auto &view = _view;
    _shader->setVec3f("view_pos", glm::vec3(view[3][0],view[3][1],view[3][2]));
    _vavbebo->bindVertexArray();
    if(_draw_type == 1){
        glDrawArrays(GL_TRIANGLES, 0, _vert_num);
    }
    else{
        glPointSize(2);
        glDrawArrays(GL_POINTS, 0, _vert_num);
    }
}


} // namespace::mlayer
