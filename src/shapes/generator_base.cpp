#include "generator_base.h"

namespace mlayer{

GeneratorBase::GeneratorBase()
{

}


GeneratorBase::~GeneratorBase()
{

}


const Vertices &GeneratorBase::vertices() const
{
    return _vertices;
}

} // namespace::mlayer