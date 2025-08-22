#ifndef FEC_ENCODER_H
#define FEC_ENCODER_H

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "bitstream.h"

/**
 * @class FECEncoder
 * @brief A class to perform Forward Error Correction (FEC) encoding on input data.
 *
 * The FECEncoder class implements convolutional coding with different rates based on the
 * baud rate and whether frequency hopping is used. It uses a constraint length 7 convolutional
 * code with repeat coding or puncturing as required.
 */
class FECEncoder {
public:
    /**
     * @brief Constructs an FECEncoder.
     * @param baud_rate The baud rate for the encoding process.
     * @param is_frequency_hopping True if frequency hopping is used, otherwise false.
     */
    FECEncoder(size_t baud_rate, bool is_frequency_hopping) 
        : baud_rate(baud_rate), is_frequency_hopping(is_frequency_hopping), shift_register(0) {}

    /**
     * @brief Encodes the input data using convolutional coding.
     * @param data The input BitStream to be encoded.
     * @return The encoded BitStream.
     */
    BitStream encode(const BitStream& data) {
        BitStream input_data(data);
        BitStream output_data;

        while (input_data.hasNext()) {
            uint8_t bit = input_data.getNextBit();
            // Shift the input bit into the shift register
            shift_register = ((shift_register << 1) | bit) & 0x7F;

            // Calculate T1 and T2 using the generator polynomials
            static inline uint8_t parity(uint8_t x) { x^=x>>4; x^=x>>2; x^=x>>1; return x&1; }
            uint8_t t1 = parity(shift_register & 0b1011011); // 133
            uint8_t t2 = parity(shift_register & 0b1111001); // 171

            // Append T1 and T2 to the encoded data
            output_data.putBit(t1);
            output_data.putBit(t2);
        }

        // Apply repetition or puncturing based on baud rate and operation mode
        return adjustRate(output_data);
    }

private:
    size_t baud_rate; ///< The baud rate used for encoding.
    bool is_frequency_hopping; ///< Indicates if frequency hopping is being used.
    uint8_t shift_register; ///< The shift register used for convolutional coding.

    /**
     * @brief Adjusts the rate of the encoded data by applying repetition or puncturing.
     * @param encoded_data The encoded BitStream to be adjusted.
     * @return The adjusted BitStream.
     */
    BitStream adjustRate(const BitStream& encoded_data) {
        BitStream adjusted_data;

        if ((baud_rate == 300 || baud_rate == 150 || baud_rate == 75) && is_frequency_hopping) {
            // Repetition for frequency-hopping operation at lower baud rates
            size_t repetition_factor = (baud_rate == 300) ? 2 : (baud_rate == 150) ? 4 : 8;
            for (size_t i = 0; i < encoded_data.getMaxBitIndex(); i += 2) {
                for (size_t j = 0; j < repetition_factor; j++) {
                    adjusted_data.putBit(encoded_data.getBitVal(i));
                    adjusted_data.putBit(encoded_data.getBitVal(i + 1));
                }
            }
        } else if ((baud_rate == 300 || baud_rate == 150) && !is_frequency_hopping) {
            // Repetition for fixed-frequency operation at lower baud rates
            size_t repetition_factor = (baud_rate == 300) ? 2 : 4;
            for (size_t i = 0; i < encoded_data.getMaxBitIndex(); i += 2) {
                for (size_t j = 0; j < repetition_factor; j++) {
                    adjusted_data.putBit(encoded_data.getBitVal(i));
                    adjusted_data.putBit(encoded_data.getBitVal(i + 1));
                }
            }
        } else {
            adjusted_data = encoded_data;
        }

        return adjusted_data;
    }
};

#endif
