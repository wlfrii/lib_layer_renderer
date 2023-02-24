#include "arrow_generator.h"
#include "cylinder_generator.h"
#include "cone_generator.h"
#include "generator_util.h"

namespace mlayer{

ArrowGenerator::ArrowGenerator(float length, float radius,
                               float cone_height, float cone_radius,
                               const glm::mat4& pose)
{
    CylinderGenerator axis(length, radius, pose);
    glm::mat4 cone_pose = pose;
    cone_pose[3][0] += length * cone_pose[2][0];
    cone_pose[3][1] += length * cone_pose[2][1];
    cone_pose[3][2] += length * cone_pose[2][2];
    ConeGenerator cone(cone_height, cone_radius, cone_pose);
    
    _vertices.clear();
    _vertices.insert(_vertices.end(), axis.vertices().begin(), axis.vertices().end());
    _vertices.insert(_vertices.end(), cone.vertices().begin(), cone.vertices().end());
}

} // namespace::mlayer