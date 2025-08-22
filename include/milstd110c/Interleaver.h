#pragma once

#include <vector>
#include "milstd110c/bitstream.h"

namespace mil::std110c {

class Interleaver {
public:
    Interleaver() = default;
    std::vector<uint8_t> interleave(const BitStream& input) const;
    static constexpr size_t kRows = 40;
    static constexpr size_t kColumns = 72;
};

} // namespace mil::std110c

