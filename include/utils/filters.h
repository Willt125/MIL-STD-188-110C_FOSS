#ifndef FILTERS_H
#define FILTERS_H

#include <cmath>
#include <cstdint>
#include <fftw3.h>
#include <numeric>
#include <vector>

class TapGenerators {
    public:
    std::vector<double> generateSRRCTaps(size_t num_taps, double sample_rate, double symbol_rate, double rolloff) const {
        std::vector<double> taps(num_taps);
        double T = 1.0 / symbol_rate; // Symbol period
        double dt = 1.0 / sample_rate; // Time step
        double t_center = (num_taps - 1) / 2.0;

        for (size_t i = 0; i < num_taps; ++i) {
            double t = (i - t_center) * dt;
            double sinc_part = (t == 0.0) ? 1.0 : std::sin(M_PI * t / T * (1 - rolloff)) / (M_PI * t / T * (1 - rolloff));
            double cos_part = (t == 0.0) ? std::cos(M_PI * t / T * (1 + rolloff)) : std::cos(M_PI * t / T * (1 + rolloff));
            double denominator = 1.0 - (4.0 * rolloff * t / T) * (4.0 * rolloff * t / T);

            if (std::fabs(denominator) < 1e-8) {
                // Handle singularity at t = T / (4R)
                taps[i] = rolloff * (std::sin(M_PI / (4.0 * rolloff)) + (1.0 / (4.0 * rolloff)) * std::cos(M_PI / (4.0 * rolloff))) / (M_PI / (4.0 * rolloff));
            } else {
                taps[i] = (4.0 * rolloff / (M_PI * std::sqrt(T))) * (cos_part / denominator);
            }

            taps[i] *= sinc_part;
        }

        // Normalize filter taps
        double sum = std::accumulate(taps.begin(), taps.end(), 0.0);
        for (auto& tap : taps) {
            tap /= sum;
        }

        return taps;
    }

    std::vector<double> generateLowpassTaps(size_t num_taps, double cutoff_freq, double sample_rate) const {
        std::vector<double> taps(num_taps);
        double fc = cutoff_freq / (sample_rate / 2.0); // Normalized cutoff frequency (0 < fc < 1)
        double M = num_taps - 1;
        double mid = M / 2.0;
    
        for (size_t n = 0; n < num_taps; ++n) {
            double n_minus_mid = n - mid;
            double h;
            if (n_minus_mid == 0.0) {
                h = fc;
            } else {
                h = fc * (std::sin(M_PI * fc * n_minus_mid) / (M_PI * fc * n_minus_mid));
            }
    
            // Apply window function (e.g., Hamming window)
            double window = 0.54 - 0.46 * std::cos(2.0 * M_PI * n / M);
            taps[n] = h * window;
        }
    
        // Normalize filter taps
        double sum = std::accumulate(taps.begin(), taps.end(), 0.0);
        for (auto& tap : taps) {
            tap /= sum;
        }
    
        return taps;
    }
};

class Filter {
    public:
    Filter(const std::vector<double>& _filter_taps) : filter_taps(_filter_taps), buffer(_filter_taps.size(), 0.0), buffer_index(0) {}

    double filterSample(const double sample) {
        buffer[buffer_index] = std::complex<double>(sample, 0.0);
        double filtered_val = 0.0;
        size_t idx = buffer_index;

        for (size_t j = 0; j < filter_taps.size(); j++) {
            filtered_val += filter_taps[j] * buffer[idx].real();
            if (idx == 0) {
                idx = buffer.size() - 1;
            } else {
                idx--;
            }
        }

        buffer_index = (buffer_index + 1) % buffer.size();
        return filtered_val;
    }

    std::complex<double> filterSample(const std::complex<double> sample) {
        buffer[buffer_index] = sample;
        std::complex<double> filtered_val = std::complex<double>(0.0, 0.0);
        size_t idx = buffer_index;
    
        for (size_t j = 0; j < filter_taps.size(); j++) {
            filtered_val += filter_taps[j] * buffer[idx];
            if (idx == 0) {
                idx = buffer.size() - 1;
            } else {
                idx--;
            }
        }
    
        buffer_index = (buffer_index + 1) % buffer.size();
        return filtered_val;
    }

    std::vector<double> applyFilter(const std::vector<double>& signal) {
        std::vector<double> filtered_signal(signal.size(), 0.0);

        // Convolve the signal with the filter taps
        for (size_t i = 0; i < signal.size(); ++i) {
            filtered_signal[i] = filterSample(signal[i]);
        }

        return filtered_signal;
    }

    std::vector<std::complex<double>> applyFilter(const std::vector<std::complex<double>>& signal) {
        std::vector<std::complex<double>> filtered_signal(signal.size(), std::complex<double>(0.0, 0.0));

        // Convolve the signal with the filter taps
        for (size_t i = 0; i < signal.size(); ++i) {
            filtered_signal[i] = filterSample(signal[i]);
        }

        return filtered_signal;
    }

    private:
    std::vector<double> filter_taps;
    std::vector<std::complex<double>> buffer;
    size_t buffer_index;
};

class SRRCFilter : public Filter {
    public:
    SRRCFilter(const size_t num_taps, const double sample_rate, const double symbol_rate, const double rolloff) : Filter(TapGenerators().generateSRRCTaps(num_taps, sample_rate, symbol_rate, rolloff)) {}
};

class LowPassFilter : public Filter {
    public:
    LowPassFilter(const size_t num_taps, const double cutoff_freq, const double sample_rate) : Filter(TapGenerators().generateLowpassTaps(num_taps, cutoff_freq, sample_rate)) {}
};

#endif /* FILTERS_H */