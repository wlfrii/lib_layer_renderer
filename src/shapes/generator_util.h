#ifndef UTIL_GENERATOR_H_LF
#define UTIL_GENERATOR_H_LF
#include <glm/glm.hpp>
#include <glm/gtx/normal.hpp>


/**
 * @brief Return the normal of a face depending on three points.
 * @param p1
 * @param p2
 * @param p3
 * @return
 */
inline glm::vec4 getNormal(const glm::vec4& p1, const glm::vec4& p2,
                           const glm::vec4& p3)
{
    const glm::vec3& t1 = p1;
    const glm::vec3& t2 = p2;
    const glm::vec3& t3 = p3;
    glm::vec3 normal = glm::triangleNormal(t1, t2, t3);
    return glm::vec4(normal, 1);
}


/**
 * @brief Return the normal of a face depending on three points.
 * @param p1
 * @param p2
 * @param p3
 * @return
 */
inline glm::vec3 getNormal(const glm::vec3& p1, const glm::vec3& p2,
                           const glm::vec3& p3)
{
    return glm::triangleNormal(p1, p2, p3);
}

#endif // UTIL_GENERATOR_H
