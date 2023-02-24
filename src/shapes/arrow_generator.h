#ifndef ARROW_GENERATOR_H_LF
#define ARROW_GENERATOR_H_LF
#include "generator_base.h"

namespace mlayer{

/**
 * @brief A class for generating vertices for arrow
 *      /\              
 *     /  \   cone_height
 *    /____\  cone_radius
 *     |  |
 *     |  |    
 *     |  |  length
 *     |__|    
 *       radius     
 */
class ArrowGenerator : public GeneratorBase
{
public:    
    /**
     * @brief Constructor of class ArrowGenerator
     * @param length       The length of axis
     * @param radius       The radius of axis
     * @param cone_height  The height of axis cone
     * @param cone_radius  The radius of axis cone
     * @param pose         The pose of coordinate
     */
    ArrowGenerator(float length, float radius, float cone_height,
                   float cone_radius, const glm::mat4& pose);

};

} // namespace::mlayer
#endif // ARROW_GENERATOR_H_LF