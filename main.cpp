// main.cpp

#include <iostream>
#include <vector>
#include <cmath>
#include <complex>
#include <random>
#include <sndfile.h> // For WAV file handling

// GNU Radio headers
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/vector_source.h>
#include <gnuradio/blocks/vector_sink.h>
#include <gnuradio/blocks/wavfile_sink.h>
#include <gnuradio/blocks/wavfile_source.h>
#include <gnuradio/blocks/multiply.h>
#include <gnuradio/blocks/complex_to_real.h>
#include <gnuradio/blocks/add_blk.h>
#include <gnuradio/analog/sig_source.h>
#include <gnuradio/analog/noise_source.h>
#include <gnuradio/filter/hilbert_fc.h>
#include <gnuradio/channels/selective_fading_model.h>

// Include your ModemController and BitStream classes
#include "ModemController.h"
#include "bitstream.h"

// Function to generate Bernoulli data
BitStream generateBernoulliData(const size_t length, const double p = 0.5, const unsigned int seed = 0) {
    BitStream random_data;
    std::mt19937 gen(seed);
    std::bernoulli_distribution dist(p);

    for (size_t i = 0; i < length * 8; ++i) {
        random_data.putBit(dist(gen));
    }
    return random_data;
}

// Function to write int16_t data to a WAV file
void writeWavFile(const std::string& filename, const std::vector<int16_t>& data, float sample_rate) {
    SF_INFO sfinfo;
    sfinfo.channels = 1;
    sfinfo.samplerate = static_cast<int>(sample_rate);
    sfinfo.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;

    SNDFILE* outfile = sf_open(filename.c_str(), SFM_WRITE, &sfinfo);
    if (!outfile) {
        std::cerr << "Error opening output file: " << sf_strerror(nullptr) << std::endl;
        return;
    }

    sf_count_t frames_written = sf_write_short(outfile, data.data(), data.size());
    if (frames_written != static_cast<sf_count_t>(data.size())) {
        std::cerr << "Error writing to output file: " << sf_strerror(outfile) << std::endl;
    }

    sf_close(outfile);
}

int main() {
    // Step 1: Gather parameters and variables

    // Define the preset based on your table (e.g., 4800 bps, 2 fading paths)
    struct ChannelPreset {
        size_t user_bit_rate;
        int num_paths;
        bool is_fading;
        float multipath_ms;
        float fading_bw_hz;
        float snr_db;
        double target_ber;
    };

    // For this example, let's use the second preset from the table
    ChannelPreset preset = {
        4800,       // user_bit_rate
        2,          // num_paths
        true,       // is_fading
        2.0f,       // multipath_ms
        0.5f,       // fading_bw_hz
        27.0f,      // snr_db
        1e-3        // target_ber
    };

    // Sampling rate (Hz)
    double Fs = 48000.0; // Adjust to match your modem's sampling rate
    double Ts = 1.0 / Fs;

    // Carrier frequency (Hz)
    float carrier_freq = 1800.0f; // Adjust to match your modem's carrier frequency

    // Step 2: Initialize the modem
    size_t baud_rate = preset.user_bit_rate;
    bool is_voice = false;
    bool is_frequency_hopping = false;
    size_t interleave_setting = 2; // Adjust as necessary

    ModemController modem(baud_rate, is_voice, is_frequency_hopping, interleave_setting);

    // Step 3: Generate input modulator data
    size_t data_length = 28800; // Length in bytes
    unsigned int data_seed = 42; // Random seed
    BitStream input_data = generateBernoulliData(data_length, 0.5, data_seed);

    // Step 4: Use the modem to modulate the input data
    std::vector<int16_t> passband_signal = modem.transmit(input_data);

    // Write the raw passband audio to a WAV file
    writeWavFile("modem_output_raw.wav", passband_signal, Fs);

    // Step 5: Process the modem output through the channel model

    // Convert passband audio to float and normalize
    std::vector<float> passband_signal_float(passband_signal.size());
    for (size_t i = 0; i < passband_signal.size(); ++i) {
        passband_signal_float[i] = passband_signal[i] / 32768.0f;
    }

    // Create GNU Radio top block
    auto tb = gr::make_top_block("Passband to Baseband and Channel Model");

    // Create vector source from passband signal
    auto src = gr::blocks::vector_source_f::make(passband_signal_float, false);

    // Apply Hilbert Transform to get analytic signal
    int hilbert_taps = 129; // Number of taps
    auto hilbert = gr::filter::hilbert_fc::make(hilbert_taps);

    // Multiply by complex exponential to shift to baseband
    auto freq_shift_down = gr::analog::sig_source_c::make(
        Fs, gr::analog::GR_COS_WAVE, -carrier_freq, 1.0f, 0.0f);

    auto multiplier_down = gr::blocks::multiply_cc::make();

    // Connect the blocks for downconversion
    tb->connect(src, 0, hilbert, 0);
    tb->connect(hilbert, 0, multiplier_down, 0);
    tb->connect(freq_shift_down, 0, multiplier_down, 1);

    // At this point, multiplier_down outputs the complex baseband signal

    // Configure the channel model parameters
    std::vector<float> delays = {0.0f};
    std::vector<float> mags = {1.0f};

    if (preset.num_paths == 2 && preset.multipath_ms > 0.0f) {
        delays.push_back(preset.multipath_ms / 1000.0f); // Convert ms to seconds
        float path_gain = 1.0f / sqrtf(2.0f); // Equal average power
        mags[0] = path_gain;
        mags.push_back(path_gain);
    }

    int N = 8;          // Number of sinusoids
    bool LOS = false;   // Rayleigh fading
    float K = 0.0f;     // K-factor
    unsigned int seed = 0;
    int ntaps = 64;     // Number of taps

    float fD = preset.fading_bw_hz; // Maximum Doppler frequency in Hz
    float fDTs = fD * Ts;           // Normalized Doppler frequency

    auto channel_model = gr::channels::selective_fading_model::make(
        N, fDTs, LOS, K, seed, delays, mags, ntaps);

    // Add AWGN to the signal
    float SNR_dB = preset.snr_db;
    float SNR_linear = powf(10.0f, SNR_dB / 10.0f);
    float signal_power = 0.0f; // Assume normalized
    for (const auto& sample : passband_signal_float) {
        signal_power += sample * sample;
    }
    signal_power /= passband_signal_float.size();
    float noise_power = signal_power / SNR_linear;
    float noise_voltage = sqrtf(noise_power);

    auto noise_src = gr::analog::noise_source_c::make(
        gr::analog::GR_GAUSSIAN, noise_voltage, seed);

    auto adder = gr::blocks::add_cc::make();

    // Connect the blocks for channel model and noise addition
    tb->connect(multiplier_down, 0, channel_model, 0);
    tb->connect(channel_model, 0, adder, 0);
    tb->connect(noise_src, 0, adder, 1);

    // Multiply by complex exponential to shift back to passband
    auto freq_shift_up = gr::analog::sig_source_c::make(
        Fs, gr::analog::GR_COS_WAVE, carrier_freq, 1.0f, 0.0f);

    auto multiplier_up = gr::blocks::multiply_cc::make();

    // Connect the blocks for upconversion
    tb->connect(adder, 0, multiplier_up, 0);
    tb->connect(freq_shift_up, 0, multiplier_up, 1);

    // Convert to real signal
    auto complex_to_real = gr::blocks::complex_to_real::make();

    // Connect the blocks
    tb->connect(multiplier_up, 0, complex_to_real, 0);

    // Collect the output samples
    auto sink = gr::blocks::vector_sink_f::make();
    tb->connect(complex_to_real, 0, sink, 0);

    // Run the flowgraph
    tb->run();

    // Retrieve the output data
    std::vector<float> received_passband_audio = sink->data();

    // Normalize and convert to int16_t
    // Find maximum absolute value
    float max_abs_value = 0.0f;
    for (const auto& sample : received_passband_audio) {
        if (fabs(sample) > max_abs_value) {
            max_abs_value = fabs(sample);
        }
    }
    if (max_abs_value == 0.0f) {
        max_abs_value = 1.0f;
    }
    float scaling_factor = 0.9f / max_abs_value; // Prevent clipping at extremes

    // Apply scaling and convert to int16_t
    std::vector<int16_t> received_passband_signal(received_passband_audio.size());
    for (size_t i = 0; i < received_passband_audio.size(); ++i) {
        float sample = received_passband_audio[i] * scaling_factor;
        // Ensure the sample is within [-1.0, 1.0]
        if (sample > 1.0f) sample = 1.0f;
        if (sample < -1.0f) sample = -1.0f;
        received_passband_signal[i] = static_cast<int16_t>(sample * 32767.0f);
    }

    // Step 6: Write the received passband audio to another WAV file
    writeWavFile("modem_output_received.wav", received_passband_signal, Fs);

    std::cout << "Processing complete. Output files generated." << std::endl;

    return 0;
}
