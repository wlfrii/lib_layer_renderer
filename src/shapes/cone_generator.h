#ifndef CONE_GENERATOR_H_LF
#define CONE_GENERATOR_H_LF
#include "generator_base.h"


/**
 * @brief A class for generating cone vertices
 *    
 *      /\              
 *     /  \   height
 *    /_  _\  radius  
 *       origin
 */
class ConeGenerator : public GeneratorBase
{
public:
    /**
     * @brief Constructor of class ConeGenerator
     * @param points_num The number of points in the cone base
     * @param height     The height of the cone
     * @param radius     The radius of the cone base
     */
    ConeGenerator(int points_num, float height, float radius);

    
    /**
     * @brief Constructor of class ConeGenerator
     * @param points_num The number of points in the cone base
     * @param pose       The pose of cone
     * @param height     The height of the cone
     * @param radius     The radius of the cone base
     */
    ConeGenerator(int points_num, const glm::mat4& pose, float height,
                  float radius);


private:
    void createVertices(int points_num, const glm::mat4 &pose, float height,
                        float radius);

    Vertices _vertices_without_base_face;
};

#endif // CONE_GENERATOR_H_LF
