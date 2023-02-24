/**--------------------------------------------------------------------
 *																		
 *   				    Layer renderer library
 *																		
 * Description:													
 * This file is a part of lib_render_layer. You can redistribute it or
 * modify it to construct your own project. It is wellcome to use this 
 * library in your scientific research work.
 * 
 * @file 		layer_util.h 
 * 
 * @brief 		The utilities for the library
 * 
 * @author		Longfei Wang
 * 
 * @date		2022/04/01
 * 
 * @license		
 * 
 * Copyright (C) 2021-Now Longfei Wang.
 * 
 * --------------------------------------------------------------------
 * Change History:                        
 * 
 * -------------------------------------------------------------------*/
#ifndef LAYER_UTIL_H_LF
#define LAYER_UTIL_H_LF
#include <glm/glm.hpp>
#include <lib_math/lib_math.h>

namespace mlayer{

/**
 * @brief Convert Eigen::Matrix4f to glm::mat4
 * @param T
 * @return
 */
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


/**
 * @brief Convert mmath::Pose to glm::mat4
 * @param pose
 * @return
 */
inline glm::mat4 cvt2GlmMat4(const mmath::Pose &pose)
{
    return cvt2GlmMat4(pose.T());
}


/**
 * @brief Convert Eigen::Vector3f to glm::mat4
 * @param vec
 * @return
 */
inline glm::vec3 cvt2GlmVec3(const Eigen::Vector3f &vec)
{
    return glm::vec3(vec[0], vec[1], vec[2]);
}


/**
 * @brief Convert glm::mat4 to mmath::Pose
 * @param mat
 * @return
 */
inline mmath::Pose cvt2Pose(const glm::mat4 &mat)
{
    mmath::Pose pose;
    pose.R(0, 0) = mat[0][0];
    pose.R(0, 1) = mat[1][0];
    pose.R(0, 2) = mat[2][0];
    pose.R(1, 0) = mat[0][1];
    pose.R(1, 1) = mat[1][1];
    pose.R(1, 2) = mat[2][1];
    pose.R(2, 0) = mat[0][2];
    pose.R(2, 1) = mat[1][2];
    pose.R(2, 2) = mat[2][2];

    pose.t[0] = mat[3][0];
    pose.t[1] = mat[3][1];
    pose.t[2] = mat[3][2];

    return pose;
}

} // namespace::mlayer
#endif // LAYER_UTIL_H_LF
