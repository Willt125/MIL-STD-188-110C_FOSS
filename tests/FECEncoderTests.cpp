#include <gtest/gtest.h>
#include "milstd110c/FECEncoder.h"

using namespace mil::std110c;

TEST(FECEncoderTests, EncodesExample)
{
    BitStream input;
    input.putBit(1);
    input.putBit(0);
    input.putBit(1);
    input.putBit(1);

    FECEncoder encoder;
    BitStream output = encoder.encode(input);

    std::vector<int> expected{1,1,1,0,1,1,1,0};
    ASSERT_EQ(output.getMaxBitIndex(), expected.size());
    for (size_t i = 0; i < expected.size(); ++i) {
        EXPECT_EQ(output.getBitVal(i), expected[i]);
    }
}

