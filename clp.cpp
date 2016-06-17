#include "clp.hpp"


template<>
std::string CommandLineParserExt::get(int index, bool space_delete) const
{
    int idx = zeroBaseIdx(index);
    return pos_args[idx];
}