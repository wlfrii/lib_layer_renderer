#ifndef GENERATORBASE_H
#define GENERATORBASE_H
#include "../layer_define.h"


class GeneratorBase
{
protected:
    GeneratorBase();
public:
    virtual ~GeneratorBase();

    const Vertices& result() const;

protected:
    Vertices    _vertices;
};

#endif // GENERATORBASE_H
