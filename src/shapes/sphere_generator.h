#ifndef CONE_GENERATOR_H_LF
#define CONE_GENERATOR_H_LF
#include "generator_base.h"

namespace mlayer{

/**
 * @brief A class for generating sphere vertices
 */
class SphereGenerator : public GeneratorBase
{
public:    
    /**
     * @brief Constructor of class ConeGenerator
     * @param radius     The radius of the sphere base
     * @param pose       The pose of sphere
     */
    SphereGenerator(float radius, const glm::mat4& pose);

};

} // namespace::mlayer
#endif // CONE_GENERATOR_H_LF
