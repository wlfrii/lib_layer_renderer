#ifndef GENERATORBASE_H
#define GENERATORBASE_H
#include "../layer_define.h"


/**
 * @brief A base class for generating vertices
 */
class GeneratorBase
{
protected:
    GeneratorBase();

public:
    virtual ~GeneratorBase();


    /**
     * @brief Return the generated vertices
     * @return
     */
    const Vertices& result() const;

protected:
    Vertices    _vertices;   //!< Store the generated vertices
};

#endif // GENERATORBASE_H
