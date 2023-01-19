#include "../include/layer_coordinate.h"
#include "../include/layer_cone.h"
#include "../include/layer_cylinder.h"
#include <gl_util.h>
#include <global.h>


LayerCoordinate::LayerCoordinate(float length, float radius,
                                 const glm::mat4& pose)
    : LayerModel(LAYER_COORDINATE, glm::vec3(0.f))
{
    float cone_radius = radius * 2.2;
    float cone_height = cone_radius * 2.5;
}


LayerCoordinate::~LayerCoordinate()
{

}


void LayerCoordinate::draw(bool is_right)
{

}





