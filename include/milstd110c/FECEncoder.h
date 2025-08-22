#pragma once

#include <cstdint>
#include "milstd110c/bitstream.h"

namespace mil::std110c {

class FECEncoder {
public:
    FECEncoder();
    BitStream encode(const BitStream& data);

private:
    uint8_t shift_register_;
};

} // namespace mil::std110c

