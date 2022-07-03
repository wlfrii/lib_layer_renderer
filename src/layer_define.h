#ifndef LAYER_DEFINE_H_LF
#define LAYER_DEFINE_H_LF
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>


/**
 * @brief A struct designed for vertex
 */
struct Vertex
{
    glm::vec4 position; //!< Positon of a vertex
    glm::vec4 normal;   //!< Normal of a vertex
};


/**
 * @brief Store the vertex data
 */
using Vertices = std::vector<Vertex>;


/**
 * @brief Store the vertex position
 */
using Positions = std::vector<glm::vec4>;




#endif // VERTEX_DATA_H_LF
