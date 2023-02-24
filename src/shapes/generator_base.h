#ifndef GENERATORBASE_H
#define GENERATORBASE_H
#include "../layer_define.h"

namespace mlayer{

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
    const Vertices& vertices() const;

protected:
    Vertices    _vertices;   //!< Store the generated vertices
};

} // namespace::mlayer
#endif // GENERATORBASE_H
