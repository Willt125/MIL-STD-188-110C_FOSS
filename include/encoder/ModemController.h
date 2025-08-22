#ifndef MODEM_CONTROLLER_H
#define MODEM_CONTROLLER_H

#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include "bitstream.h"
#include "FECEncoder.h"
#include "Interleaver.h"
#include "MGDDecoder.h"
#include "PSKModulator.h"
#include "Scrambler.h"
#include "SymbolFormation.h"

/**
 * @brief Clamps an integer value to the range of int16_t.
 * @param x The value to be clamped.
 * @return The clamped value.
 */
constexpr int16_t clamp(int16_t x) {
    constexpr int16_t max_val = std::numeric_limits<int16_t>::max();
    constexpr int16_t min_val = std::numeric_limits<int16_t>::min();
    return (x > max_val) ? max_val : (x < min_val) ? min_val : x;
}

/**
 * @class ModemController
 * @brief Controls the modulation process for transmitting data using FEC encoding, interleaving, scrambling, and PSK modulation.
 */
class ModemController {
public:
    /**
     * @brief Constructs a ModemController object.
     * @param baud_rate The baud rate for the modem.
     * @param is_voice Indicates if the data being transmitted is voice.
     * @param is_frequency_hopping Indicates if frequency hopping is used.
     * @param interleave_setting The interleave setting to be used.
     * @param data The input data stream to be transmitted. The `is_voice` parameter controls whether the modem treats it as binary file data,
     *             or a binary stream from the MELPe (or other) voice codec.
     */
    ModemController(const size_t _baud_rate, const bool _is_voice, const bool _is_frequency_hopping, const size_t _interleave_setting)
        : baud_rate(_baud_rate),
          is_voice(_is_voice),
          is_frequency_hopping(_is_frequency_hopping),
          interleave_setting(_interleave_setting),
          symbol_formation(_baud_rate, _interleave_setting, _is_voice, _is_frequency_hopping),
          scrambler(),
          fec_encoder(_baud_rate, _is_frequency_hopping),
          interleaver(_baud_rate, _interleave_setting, _is_frequency_hopping),
          mgd_decoder(_baud_rate, _is_frequency_hopping),
          modulator(48000, _is_frequency_hopping, 48) {}

    /**
     * @brief Transmits the input data by processing it through different phases like FEC encoding, interleaving, symbol formation, scrambling, and modulation.
     * @return The scrambled data ready for modulation.
     * @note The modulated signal is generated internally but is intended to be handled externally.
     */
    std::vector<int16_t> transmit(const BitStream& input_data) {
        // Step 1: Append EOM Symbols
        BitStream raw = buildRawMessageWithEOM(input_data);

        std::vector<uint8_t> processed_symbols;
        if (baud_rate == 4800) {
            processed_symbols = splitTribitSymbols(raw);
        } else {
            // Step 2: FEC
            BitStream fec_bits = fec_encoder.encode(raw);

            // Step 3: Interleaving (+ implicit interleaver pad handled in interleaver)
            processed_symbols = interleaver.interleaveStream(fec_bits);
        }
        // Step 4: MGD map
        auto mgd_decoded = mgd_decoder.mgdDecode(processed_symbols);

        // Step 5: Symbol Formation. This function injects the sync preamble symbols.
		// Scrambling is handled internally.
        auto symbol_stream = symbol_formation.formSymbols(mgd_decoded);

        // Step 6. Modulate to passband.
        return modulator.modulate(symbol_stream);
    }

    BitStream receive(const std::vector<int16_t>& passband_signal) {
        // Step one: Demodulate the passband signal and retrieve decoded symbols
        std::vector<uint8_t> demodulated_symbols = modulator.demodulate(passband_signal, baud_rate, interleave_setting, is_voice);

        return BitStream();
    }

private:
    size_t baud_rate;                ///< The baud rate for the modem.
    bool is_voice;                   ///< Indicates if the data being transmitted is voice.
    bool is_frequency_hopping;       ///< Indicates if frequency hopping is used.
    size_t interleave_setting;       ///< The interleave setting to be used.
    size_t sample_rate;

    SymbolFormation symbol_formation; ///< Symbol formation instance to form symbols from data.
    Scrambler scrambler;              ///< Scrambler instance for scrambling the data.
    FECEncoder fec_encoder;           ///< FEC encoder instance for encoding the data.
    Interleaver interleaver;          ///< Interleaver instance for interleaving the data.
    PSKModulator modulator;           ///< PSK modulator instance for modulating the data.
    MGDDecoder mgd_decoder;           ///< MGD decoder

    BitStream buildRawMessageWithEOM(const BitStream& input_data) {
    	BitStream bs = input_data;
    	
    	// 32-bit EOM, MSB first
    	constexpr uint32_t EOM = 0x4B65A5B2u;
    	bs.putU32MSB(EOM);
    	
    	// Convolutional encoder tail: K-1 = 6 zeros for K=7
    	for (int i = 0; i < 6; ++i) bs.putBit(0);
    	
    	return bs;
    }

    std::vector<uint8_t> splitTribitSymbols(const BitStream& input_data) {
        std::vector<uint8_t> return_vector;
        size_t max_index = input_data.getMaxBitIndex();
        size_t current_index = 0;

        while (current_index + 2 <= max_index) {
            uint8_t symbol = 0;
            for (int i = 0; i < 3; i++) {
                symbol = (symbol << 1) | input_data.getBitVal(current_index + i);
            }
            return_vector.push_back(symbol);
            current_index += 3;
        }

        return return_vector;
    }
};



#endif
