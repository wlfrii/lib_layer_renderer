#ifndef SEGMENT_GENERATOR_H_LF
#define SEGMENT_GENERATOR_H_LF
#include "generator_base.h"

class SegmentGenerator : public GeneratorBase
{
public:
    SegmentGenerator();
    SegmentGenerator(int points_num, float length, float len_gap,
                     float theta, float delta, float radius);

    const Vertices& resultSpacers() const;

private:
    Vertices _vertices_spacers;
};

#endif // SEGMENT_GENERATOR_H_LF
