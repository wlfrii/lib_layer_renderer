#include "../export/lib_layer_renderer/layer_background3d.h"
#include <gl_util.h>

namespace mlayer{


LayerBackground3D::LayerBackground3D(float bgwidth, float bgheight, float bgdepth)
    : LayerBackground(bgwidth, bgheight, bgdepth)
{
}


LayerBackground3D::~LayerBackground3D()
{

}



void LayerBackground3D::setModel(const glm::mat4& model)
{
    _model = model;
}


void LayerBackground3D::setView(const glm::mat4& view)
{
    _view = view;
}


void LayerBackground3D::setProjection(const glm::mat4& projection)
{
    _projection = projection;
}


void LayerBackground3D::draw()
{
    _shader->use();
    _shader->setMat4f("projection", _projection);
    _shader->setMat4f("model", _global * _model);
    _shader->setMat4f("view", _view);
    _vavbebo->bindVertexArray();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, _texture);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, _texture_mask);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

} // namespace::mlayer
