#ifndef CONE_GENERATOR_H_LF
#define CONE_GENERATOR_H_LF
#include "generator_base.h"

namespace mlayer{

/**
 * @brief A class for generating cone vertices
 *    
 *      /\              
 *     /  \   height
 *    /_. _\  radius
 *       origin
 */
class ConeGenerator : public GeneratorBase
{
public:    
    /**
     * @brief Constructor of class ConeGenerator
     * @param height     The height of the cone
     * @param radius     The radius of the cone base
     * @param pose       The pose of cone
     */
    ConeGenerator(float height, float radius, const glm::mat4& pose);

};

} // namespace::mlayer
#endif // CONE_GENERATOR_H_LF