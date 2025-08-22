#ifndef WATTERSONCHANNEL_H
#define WATTERSONCHANNEL_H

#include <iostream>
#include <complex>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <functional>
#include <fftw3.h> // FFTW library for FFT-based Hilbert transform

constexpr double PI = 3.14159265358979323846;

class WattersonChannel {
public:
    WattersonChannel(double sampleRate, double symbolRate, double delaySpread, double fadingBandwidth, double SNRdB, int numSamples, int numpaths, bool isFading);

    // Process a block of input samples
    void process(const std::vector<double>& inputSignal, std::vector<double>& outputSignal);

private:
    double Fs; // Sample rate
    double Rs; // Symbol rate
    double delaySpread; // Delay spread in seconds
    std::vector<int> delays = {0, L};
    double fadingBandwidth; // Fading bandwidth d in Hz
    double SNRdB; // SNR in dB
    int L; // Length of the simulated channel
    std::vector<double> f_jt; // Filter impulse response
    std::vector<std::vector<std::complex<double>>> h_j; // Fading tap gains over time for h_0 and h_(L-1)
    double Ts; // Sample period
    double k; // Normalization constant for filter
    double tau; // Truncation width
    double fadingSampleRate; // Sample rate for fading process
    std::vector<std::vector<double>> wgnFadingReal; // WGN samples for fading (double part)
    std::vector<std::vector<double>> wgnFadingImag; // WGN samples for fading (imaginary part)
    std::vector<std::complex<double>> n_i; // WGN samples for noise
    std::mt19937 rng; // Random number generator
    int numSamples; // Number of samples in the simulation
    int numFadingSamples; // Number of fading samples

    int numPaths;
    bool isFading;

    void normalizeTapGains();
    void generateFilter();
    void generateFadingTapGains();
    void generateNoise(const std::vector<std::complex<double>>& x_i);
    void generateWGN(std::vector<double>& wgn, int numSamples);
    void resampleFadingTapGains();
    void hilbertTransform(const std::vector<double>& input, std::vector<std::complex<double>>& output);
};

WattersonChannel::WattersonChannel(double sampleRate, double symbolRate, double delaySpread, double fadingBandwidth, double SNRdB, int numSamples, int numPaths, bool isFading)
    : Fs(sampleRate), Rs(symbolRate), delaySpread(delaySpread), fadingBandwidth(fadingBandwidth), SNRdB(SNRdB), numSamples(numSamples), rng(std::random_device{}()), numPaths(numPaths), isFading(isFading)
{
    Ts = 1.0 / Fs;
    // Compute L
    if (numPaths == 1) {
        L = 1;
    } else {
        L = static_cast<int>(std::round(delaySpread / Ts));
        if (L < 1) L = 1;
    }

    // Compute truncation width tau
    double ln100 = std::log(100.0);
    tau = std::sqrt(ln100) / (PI * fadingBandwidth);

    // Initialize k (will be normalized later)
    k = 1.0;

    // Fading sample rate, at least 32 times the fading bandwidth
    fadingSampleRate = std::max(32.0 * fadingBandwidth, Fs);

    h_j.resize(numPaths);
    wgnFadingReal.resize(numPaths);
    wgnFadingImag.resize(numPaths);

    if (isFading) {
        // Generate filter impulse response
        generateFilter();

        // Number of fading samples
        double simulationTime = numSamples / Fs;
        numFadingSamples = static_cast<int>(std::ceil(simulationTime * fadingSampleRate));

        // Generate WGN for fading
        for (int pathIndex = 0; pathIndex < numPaths; ++pathIndex) {
        generateWGN(wgnFadingReal[pathIndex], numFadingSamples);
        generateWGN(wgnFadingImag[pathIndex], numFadingSamples);
        }

        // Generate fading tap gains
        generateFadingTapGains();

        // Resample fading tap gains to match sample rate Fs
        resampleFadingTapGains();
    } else {
        // For fixed channel, set tap gains directly
        generateFadingTapGains();
    }

    // Generate noise n_i
}

void WattersonChannel::normalizeTapGains() {
    double totalPower = 0.0;
    int numValidSamples = h_j[0].size();
    for (int i = 0; i < numValidSamples; i++) {
        for (int pathIndex = 0; pathIndex < numPaths; pathIndex++) {
            totalPower += std::norm(h_j[pathIndex][i]);
        }
    }
    totalPower /= numValidSamples;

    double normFactor = 1.0 / std::sqrt(totalPower);
    for (int pathIndex = 0; pathIndex < numPaths; pathIndex++) {
        for (auto& val : h_j[pathIndex]) {
            val *= normFactor;
        }
    }
}

void WattersonChannel::generateFilter()
{
    // Generate filter impulse response f_j(t) = k * sqrt(2) * e^{-π² * t² * d²}, -tau < t < tau

    // Number of filter samples
    int numFilterSamples = static_cast<int>(std::ceil(2 * tau * fadingSampleRate)) + 1; // Include center point
    f_jt.resize(numFilterSamples);

    double dt = 1.0 / fadingSampleRate;
    int halfSamples = numFilterSamples / 2;

    double totalEnergy = 0.0;

    for (int n = 0; n < numFilterSamples; ++n) {
        double t = (n - halfSamples) * dt;
        double val = k * std::sqrt(2.0) * std::exp(-PI * PI * t * t * fadingBandwidth * fadingBandwidth);
        f_jt[n] = val;
        totalEnergy += val * val * dt;
    }

    // Normalize k so that total energy is 1.0
    double k_new = k / std::sqrt(totalEnergy);
    for (auto& val : f_jt) {
        val *= k_new;
    }
    k = k_new;
}

void WattersonChannel::generateFadingTapGains()
{
    if (!isFading) {
        for (int pathIndex = 0; pathIndex < numPaths; pathIndex++) {
            h_j[pathIndex].assign(numSamples, std::complex<double>(1.0, 0.0));
        }
    } else {
        // Prepare for FFT-based convolution
        int convSize = numFadingSamples + f_jt.size() - 1;
        int fftSize = 1;
        while (fftSize < convSize) {
            fftSize <<= 1; // Next power of two
        }

        std::vector<double> f_jtPadded(fftSize, 0.0);
        std::copy(f_jt.begin(), f_jt.end(), f_jtPadded.begin());

        fftw_complex* f_jtFFT = fftw_alloc_complex(fftSize);
        fftw_plan planF_jt = fftw_plan_dft_r2c_1d(fftSize, f_jtPadded.data(), f_jtFFT, FFTW_ESTIMATE);
        fftw_execute(planF_jt);

        for (int pathIndex = 0; pathIndex < numPaths; pathIndex++) {
            // Zero-pad inputs
            std::vector<double> wgnRealPadded(fftSize, 0.0);
            std::vector<double> wgnImagPadded(fftSize, 0.0);
            
            std::copy(wgnFadingReal[pathIndex].begin(), wgnFadingReal[pathIndex].end(), wgnRealPadded.begin());
            std::copy(wgnFadingImag[pathIndex].begin(), wgnFadingImag[pathIndex].end(), wgnImagPadded.begin());

            // Perform FFTs
            fftw_complex* WGNRealFFT = fftw_alloc_complex(fftSize);
            fftw_complex* WGNImagFFT = fftw_alloc_complex(fftSize);
            fftw_plan planWGNReal = fftw_plan_dft_r2c_1d(fftSize, wgnRealPadded.data(), WGNRealFFT, FFTW_ESTIMATE);
            fftw_plan planWGNImag = fftw_plan_dft_r2c_1d(fftSize, wgnImagPadded.data(), WGNImagFFT, FFTW_ESTIMATE);

            fftw_execute(planWGNReal);
            fftw_execute(planWGNImag);

            // Multiply in frequency domain
            int fftComplexSize = fftSize / 2 + 1;
            for (int i = 0; i < fftComplexSize; ++i) {
                // Multiply WGNRealFFT and f_jtFFT
                double realPart = WGNRealFFT[i][0] * f_jtFFT[i][0] - WGNRealFFT[i][1] * f_jtFFT[i][1];
                double imagPart = WGNRealFFT[i][0] * f_jtFFT[i][1] + WGNRealFFT[i][1] * f_jtFFT[i][0];
                WGNRealFFT[i][0] = realPart;
                WGNRealFFT[i][1] = imagPart;

                // Multiply WGNImagFFT and f_jtFFT
                realPart = WGNImagFFT[i][0] * f_jtFFT[i][0] - WGNImagFFT[i][1] * f_jtFFT[i][1];
                imagPart = WGNImagFFT[i][0] * f_jtFFT[i][1] + WGNImagFFT[i][1] * f_jtFFT[i][0];
                WGNImagFFT[i][0] = realPart;
                WGNImagFFT[i][1] = imagPart;
            }

            // Perform inverse FFTs
            fftw_plan planInvReal = fftw_plan_dft_c2r_1d(fftSize, WGNRealFFT, wgnRealPadded.data(), FFTW_ESTIMATE);
            fftw_plan planInvImag = fftw_plan_dft_c2r_1d(fftSize, WGNImagFFT, wgnImagPadded.data(), FFTW_ESTIMATE);

            fftw_execute(planInvReal);
            fftw_execute(planInvImag);

            // Normalize
            double scale = 1.0 / fftSize;
            for (int i = 0; i < fftSize; ++i) {
                wgnRealPadded[i] *= scale;
                wgnImagPadded[i] *= scale;
            }

            // Assign h_j[0] and h_j[1]
            int numValidSamples = numFadingSamples;

            h_j[pathIndex].resize(numValidSamples);
            for (int i = 0; i < numValidSamples; i++) {
                h_j[pathIndex][i] = std::complex<double>(wgnRealPadded[i], wgnImagPadded[i]);
            }

            // Clean up
            fftw_destroy_plan(planWGNReal);
            fftw_destroy_plan(planWGNImag);
            fftw_destroy_plan(planInvReal);
            fftw_destroy_plan(planInvImag);
            fftw_free(WGNRealFFT);
            fftw_free(WGNImagFFT);
        }

        fftw_destroy_plan(planF_jt);
        fftw_free(f_jtFFT);

        normalizeTapGains();
    }
}

void WattersonChannel::resampleFadingTapGains()
{
    // Resample h_j[0] and h_j[1] from fadingSampleRate to Fs
    int numOutputSamples = numSamples;
    double resampleRatio = fadingSampleRate / Fs;

    for (int pathIndex = 0; pathIndex < numPaths; pathIndex++) {
        std::vector<std::complex<double>> resampled_h(numOutputSamples);
        for (int i = 0; i < numOutputSamples; ++i) {
            double t = i * (1.0 / Fs);
            double index = t * fadingSampleRate;
            int idx = static_cast<int>(index);
            double frac = index - idx;

            // Simple linear interpolation
            if (idx + 1 < h_j[pathIndex].size()) {
                resampled_h[i] = h_j[pathIndex][idx] * (1.0 - frac) + h_j[pathIndex][idx + 1] * frac;
            }
            else if (idx < h_j[pathIndex].size()) {
                resampled_h[i] = h_j[pathIndex][idx];
            }
            else {
                resampled_h[i] = std::complex<double>(0.0, 0.0);
            }
        }
        h_j[pathIndex] = std::move(resampled_h);
    }
}

void WattersonChannel::generateNoise(const std::vector<std::complex<double>>& x_i)
{
    // Generate WGN samples for noise n_i with appropriate power to achieve the specified SNR
    n_i.resize(numSamples);

    double inputSignalPower = 0.0;
    for (const auto& sample : x_i) {
        inputSignalPower += std::norm(sample);
    }
    inputSignalPower /= x_i.size();

    // Compute signal power (assuming average power of input signal x_i is normalized to 1.0)
    double channelGainPower = 0.0;
    for (int i = 0; i < numSamples; i++) {
        std::complex<double> combinedGain = std::complex<double>(0.0, 0.0);
        for (int pathIndex = 0; pathIndex < numPaths; pathIndex++) {
            combinedGain += h_j[pathIndex][i];
        }
        channelGainPower += std::norm(combinedGain);
    }
    channelGainPower /= numSamples;

    double signalPower = inputSignalPower * channelGainPower;

    // Compute noise power
    double SNR_linear = std::pow(10.0, SNRdB / 10.0);
    double noisePower = signalPower / SNR_linear;

    std::normal_distribution<double> normalDist(0.0, std::sqrt(noisePower / 2.0)); // Divided by 2 for double and imag parts

    for (int i = 0; i < numSamples; ++i) {
        double realPart = normalDist(rng);
        double imagPart = normalDist(rng);
        n_i[i] = std::complex<double>(realPart, imagPart);
    }
}

void WattersonChannel::generateWGN(std::vector<double>& wgn, int numSamples)
{
    wgn.resize(numSamples);

    std::normal_distribution<double> normalDist(0.0, 1.0); // Standard normal distribution

    for (int i = 0; i < numSamples; ++i) {
        wgn[i] = normalDist(rng);
    }
}

void WattersonChannel::hilbertTransform(const std::vector<double>& input, std::vector<std::complex<double>>& output)
{
    // Implement Hilbert transform using FFT method
    int N = input.size();

    // Allocate input and output arrays for FFTW
    double* in = fftw_alloc_real(N);
    fftw_complex* out = fftw_alloc_complex(N);

    // Copy input signal to in array
    for (int i = 0; i < N; ++i) {
        in[i] = input[i];
    }

    // Create plan for forward FFT
    fftw_plan plan_forward = fftw_plan_dft_r2c_1d(N, in, out, FFTW_ESTIMATE);

    // Execute forward FFT
    fftw_execute(plan_forward);

    // Apply the Hilbert transform in frequency domain
    // For positive frequencies, multiply by 2; for zero and negative frequencies, set to zero
    int N_half = N / 2 + 1;
    for (int i = 0; i < N_half; ++i) {
        if (i == 0 || i == N / 2) { // DC and Nyquist frequency components
            out[i][0] = 0.0;
            out[i][1] = 0.0;
        }
        else {
            out[i][0] *= 2.0;
            out[i][1] *= 2.0;
        }
    }

    // Create plan for inverse FFT
    fftw_plan plan_backward = fftw_plan_dft_c2r_1d(N, out, in, FFTW_ESTIMATE);

    // Execute inverse FFT
    fftw_execute(plan_backward);

    // Normalize and store result in output vector
    output.resize(N);
    double scale = 1.0 / N;
    for (int i = 0; i < N; ++i) {
        output[i] = std::complex<double>(input[i], in[i] * scale);
    }

    // Clean up
    fftw_destroy_plan(plan_forward);
    fftw_destroy_plan(plan_backward);
    fftw_free(in);
    fftw_free(out);
}

void WattersonChannel::process(const std::vector<double>& inputSignal, std::vector<double>& outputSignal)
{
    // Apply Hilbert transform to input signal to get complex x_i
    std::vector<std::complex<double>> x_i;
    hilbertTransform(inputSignal, x_i);

    generateNoise(x_i);

    // Process the signal through the channel
    std::vector<std::complex<double>> y_i(numSamples);

    // For each sample, compute y_i = h_j[0][i] * x_i + h_j[1][i] * x_{i - (L - 1)} + n_i[i]
    for (int i = 0; i < numSamples; ++i) {
        std::complex<double> y = n_i[i];

        for (int pathIndex = 0; pathIndex < numPaths; pathIndex++) {
            int delay = delays[pathIndex];
            int idx = i - delay;
            if (idx >= 0) {
                y += h_j[pathIndex][i] * x_i[idx];
            }
        }

        y_i[i] = y;
    }

    // Output the double part of y_i
    outputSignal.resize(numSamples);
    for (int i = 0; i < numSamples; ++i) {
        outputSignal[i] = y_i[i].real();
    }
}

#endif