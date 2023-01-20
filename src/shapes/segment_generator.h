#ifndef SEGMENT_GENERATOR_H_LF
#define SEGMENT_GENERATOR_H_LF
#include "generator_base.h"


/**
 * @brief A class for generating continuum segment vertices
 */
class SegmentGenerator : public GeneratorBase
{
public:
    /**
     * @brief Default constructor of class SegmentGenerator
     */
    SegmentGenerator();


    /**
     * @brief Constructor of class SegmentGenerator.
     * @param length     The length of the segment
     * @param len_gap    The length gap of the segment
     * @param theta      The bending angle of the segment
     * @param delta      The bending direction of the segment
     * @param radius     The radius of the segment
     */
    SegmentGenerator(float length, float len_gap,
                     float theta, float delta, float radius);


    /**
     * @brief Reture the vertices of the spacers of the segment.
     * @return
     */
    const Vertices& spacerVertices() const;

private:
    Vertices _vertices_spacers;
};

#endif // SEGMENT_GENERATOR_H_LF
