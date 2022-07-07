#ifndef LAYER_UTIL_H_LF
#define LAYER_UTIL_H_LF
#include <glm/glm.hpp>
#include <lib_math/lib_math.h>

inline glm::mat4 cvt2GlmMat4(const Eigen::Matrix4f &T)
{
    glm::mat4 mat(1.0);
    mat[0][0] = T(0, 0);
    mat[1][0] = T(0, 1);
    mat[2][0] = T(0, 2);
    mat[0][1] = T(1, 0);
    mat[1][1] = T(1, 1);
    mat[2][1] = T(1, 2);
    mat[0][2] = T(2, 0);
    mat[1][2] = T(2, 1);
    mat[2][2] = T(2, 2);

    mat[3][0] = T(0, 3);
    mat[3][1] = T(1, 3);
    mat[3][2] = T(2, 3);

    return mat;
}


inline glm::mat4 cvt2GlmMat4(const mmath::Pose &pose)
{
    return cvt2GlmMat4(pose.T());
}


inline glm::vec3 cvt2GlmVec3(const Eigen::Vector3f &vec)
{
    return glm::vec3(vec[0], vec[1], vec[2]);
}


#endif // LAYER_UTIL_H_LF
